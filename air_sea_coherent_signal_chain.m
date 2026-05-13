function chain = air_sea_coherent_signal_chain(cfg)
% 空海平台多节点高效相参合成方法：信号处理逻辑链路
% 六阶段：任务牵引 -> 误差建模 -> 时空统一 -> 分层补偿 -> 自适应融合 -> 闭环优化

if nargin < 1
    cfg = struct();
end

cfg = fill_default_cfg(cfg);
validate_cfg(cfg);

stage1 = task_driven_stage(cfg.mission, cfg.nodes);
stage2 = error_modeling_stage(cfg.errors, stage1);
stage3 = spacetime_unification_stage(cfg.sync, stage2, cfg.nodes);
stage4 = layered_compensation_stage(cfg.compensation, stage3);
stage5 = adaptive_fusion_stage(cfg.fusion, cfg.nodes, stage4);
stage6 = closed_loop_stage(cfg.optimization, stage5);

chain = struct();
chain.task_driven = stage1;
chain.error_modeling = stage2;
chain.spacetime_unification = stage3;
chain.layered_compensation = stage4;
chain.adaptive_fusion = stage5;
chain.closed_loop = stage6;
end

function cfg = fill_default_cfg(cfg)
cfg = set_default(cfg, 'mission', struct( ...
    'mode', 'detect_imaging', ...
    'snr_target_db', 12, ...
    'cross_node_phase_target_deg', 8, ...
    'max_latency_s', 0.08));

cfg = set_default(cfg, 'nodes', struct( ...
    'link_quality', [0.88 0.81 0.76 0.72], ...
    'credibility', [0.93 0.90 0.86 0.79], ...
    'platform_type', {{'air','air','sea','sea'}}));

cfg = set_default(cfg, 'errors', struct( ...
    'time_sync_ns', [12 17 20 24], ...
    'freq_offset_hz', [1.1 1.5 2.0 2.4], ...
    'phase_drift_deg', [3.5 4.0 5.4 6.1], ...
    'position_m', [2.4 3.0 3.8 4.5], ...
    'channel_fade_db', [0.6 0.8 1.1 1.3]));

cfg = set_default(cfg, 'sync', struct( ...
    'time_reference_stability', 0.91, ...
    'geometry_alignment', 0.87));

cfg = set_default(cfg, 'compensation', struct( ...
    'coarse_sync_factor', 0.58, ...
    'fine_sync_factor', 0.73, ...
    'prediction_factor', 0.56, ...
    'feedback_factor', 0.62));

cfg = set_default(cfg, 'fusion', struct( ...
    'selection_threshold', 0.62, ...
    'weight_exponent', 1.3));

cfg = set_default(cfg, 'optimization', struct( ...
    'snr_gain_target_db', 5.5, ...
    'sidelobe_target_db', 9.5, ...
    'robustness_target', 0.80, ...
    'realtime_target', 0.85));
end

function stage = task_driven_stage(mission, nodes)
required_phase_rad = deg2rad(mission.cross_node_phase_target_deg);
precision_index = clamp(1 - required_phase_rad / deg2rad(30), 0.05, 1.0);
coordination_mode = 'hybrid_coherent_cluster';
if strcmpi(mission.mode, 'detect_only')
    coordination_mode = 'coherent_priority_detection';
elseif strcmpi(mission.mode, 'imaging_only')
    coordination_mode = 'coherent_priority_imaging';
end

stage = struct();
stage.required_phase_rad = required_phase_rad;
stage.required_snr_db = mission.snr_target_db;
stage.max_latency_s = mission.max_latency_s;
stage.precision_index = precision_index;
stage.coordination_mode = coordination_mode;
stage.available_nodes = numel(nodes.link_quality);
end

function stage = error_modeling_stage(errors, stage1)
n = numel(errors.time_sync_ns);
time_norm = normalize_metric(errors.time_sync_ns, 40);
freq_norm = normalize_metric(errors.freq_offset_hz, 5);
phase_norm = normalize_metric(errors.phase_drift_deg, 20);
pos_norm = normalize_metric(errors.position_m, 8);
channel_norm = normalize_metric(errors.channel_fade_db, 3);

err_vector = (0.24 * time_norm + 0.20 * freq_norm + 0.28 * phase_norm + ...
    0.16 * pos_norm + 0.12 * channel_norm);

model_confidence = clamp(1 - mean(err_vector) * (1 + 0.2 * (1 - stage1.precision_index)), 0, 1);

stage = struct();
stage.node_count = n;
stage.unified_error_vector = err_vector;
stage.model_confidence = model_confidence;
stage.mean_error = mean(err_vector);
stage.max_error = max(err_vector);
end

function stage = spacetime_unification_stage(sync, stage2, nodes)
q_link = nodes.link_quality(:)';
quality_alignment = mean(q_link) * sync.geometry_alignment;
sync_quality = sync.time_reference_stability * (1 - stage2.mean_error);
unified_score = clamp(0.5 * quality_alignment + 0.5 * sync_quality, 0, 1);

stage = struct();
stage.time_base_quality = sync.time_reference_stability;
stage.geometry_alignment = sync.geometry_alignment;
stage.quality_alignment = quality_alignment;
stage.sync_quality = sync_quality;
stage.unified_spacetime_score = unified_score;
end

function stage = layered_compensation_stage(compensation, stage3)
coarse_gain = clamp(compensation.coarse_sync_factor * stage3.unified_spacetime_score, 0, 1);
fine_gain = clamp(compensation.fine_sync_factor * (0.5 + 0.5 * stage3.sync_quality), 0, 1);
predictive_gain = clamp(compensation.prediction_factor * stage3.unified_spacetime_score, 0, 1);
feedback_gain = clamp(compensation.feedback_factor * stage3.sync_quality, 0, 1);

residual_ratio = clamp(1 - (0.35 * coarse_gain + 0.35 * fine_gain + ...
    0.15 * predictive_gain + 0.15 * feedback_gain), 0.02, 1.0);

stage = struct();
stage.coarse_gain = coarse_gain;
stage.fine_gain = fine_gain;
stage.predictive_gain = predictive_gain;
stage.feedback_gain = feedback_gain;
stage.residual_ratio = residual_ratio;
end

function stage = adaptive_fusion_stage(fusion, nodes, stage4)
link_quality = nodes.link_quality(:)';
credibility = nodes.credibility(:)';
n = numel(link_quality);

base = (link_quality .* credibility) .^ fusion.weight_exponent;
penalty = (1 - stage4.residual_ratio);
weights = normalize_weights(base * penalty);

selected = (weights >= fusion.selection_threshold / max(n, 1));
if ~any(selected)
    [~, idx] = max(weights);
    selected(idx) = true;
end

effective_weights = weights .* selected;
effective_weights = normalize_weights(effective_weights);

fusion_efficiency = clamp(sum(effective_weights .* (link_quality .* credibility)), 0, 1);
snr_gain_db = 10 * log10(max(1, 1 + n * fusion_efficiency * (1 - stage4.residual_ratio)));

stage = struct();
stage.node_weights = effective_weights;
stage.selected_nodes = selected;
stage.fusion_efficiency = fusion_efficiency;
stage.estimated_snr_gain_db = snr_gain_db;
end

function stage = closed_loop_stage(opt, stage5)
sidelobe_suppression = clamp(6 + 8 * stage5.fusion_efficiency, 0, 30);
robustness = clamp(0.55 + 0.4 * stage5.fusion_efficiency, 0, 1);
realtime = clamp(0.92 - 0.12 * (1 - stage5.fusion_efficiency), 0, 1);

snr_gap = opt.snr_gain_target_db - stage5.estimated_snr_gain_db;
sidelobe_gap = opt.sidelobe_target_db - sidelobe_suppression;
robust_gap = opt.robustness_target - robustness;
rt_gap = opt.realtime_target - realtime;

stage = struct();
stage.metrics = struct( ...
    'snr_gain_db', stage5.estimated_snr_gain_db, ...
    'sidelobe_suppression_db', sidelobe_suppression, ...
    'robustness', robustness, ...
    'realtime', realtime);
stage.gaps = struct( ...
    'snr_gap_db', snr_gap, ...
    'sidelobe_gap_db', sidelobe_gap, ...
    'robustness_gap', robust_gap, ...
    'realtime_gap', rt_gap);
stage.next_iteration = struct( ...
    'compensation_factor', clamp(0.6 + 0.08 * snr_gap + 0.05 * robust_gap, 0.3, 0.95), ...
    'fusion_threshold', clamp(0.58 + 0.04 * sidelobe_gap, 0.45, 0.80), ...
    'clock_tightening_factor', clamp(1 + 0.08 * max(0, snr_gap), 1.0, 1.5));
end

function cfg = set_default(cfg, field_name, value)
if ~isfield(cfg, field_name) || isempty(cfg.(field_name))
    cfg.(field_name) = value;
else
    cfg.(field_name) = merge_struct(value, cfg.(field_name));
end
end

function out = merge_struct(base, override)
out = base;
if isempty(override)
    return;
end
f = fieldnames(override);
for i = 1:numel(f)
    key = f{i};
    if isfield(base, key) && isstruct(base.(key)) && isstruct(override.(key))
        out.(key) = merge_struct(base.(key), override.(key));
    else
        out.(key) = override.(key);
    end
end
end

function validate_cfg(cfg)
need = {'mission','nodes','errors','sync','compensation','fusion','optimization'};
for i = 1:numel(need)
    if ~isfield(cfg, need{i})
        error('cfg.%s is required.', need{i});
    end
end

ln = numel(cfg.nodes.link_quality);
if ln == 0
    error('cfg.nodes.link_quality can not be empty.');
end
if numel(cfg.nodes.credibility) ~= ln || numel(cfg.errors.time_sync_ns) ~= ln || ...
        numel(cfg.errors.freq_offset_hz) ~= ln || numel(cfg.errors.phase_drift_deg) ~= ln || ...
        numel(cfg.errors.position_m) ~= ln || numel(cfg.errors.channel_fade_db) ~= ln
    error('Node-related arrays must have same length.');
end
end

function y = normalize_metric(x, scale)
y = clamp(abs(x) ./ max(scale, eps), 0, 1);
end

function w = normalize_weights(x)
s = sum(x);
if s <= eps
    w = ones(size(x)) / numel(x);
else
    w = x / s;
end
end

function y = clamp(x, lo, hi)
y = min(max(x, lo), hi);
end
