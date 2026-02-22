function generated = generateBoulicTargetsFromParaCfg(paraCfg)
%GENERATEBOULICTARGETSFROMPARACFG Optional Boulic target file generation.
% When enabled by paraCfg fields, this function creates TargetBase*.dat and
% TargetJoints*.dat in the scenario Input/ folder using a Boulic-style
% kinematic walking model (Boulic et al., 1989).
%
% Enable with:
% - switchGenerateBoulicTargets = 1
%
% Required parameter:
% - boulicNumberOfTargets
%
% Optional parameters (all in paraCfgCurrent.txt):
% - boulicTemplateScenario       (default: examples/SensingTwoTargets)
% - boulicStartXRange            (default: [-2, 2])
% - boulicStartYRange            (default: [-2, 2])
% - boulicVelocityRange          (default: [0.6, 1.4]) m/s
% - boulicDirectionRangeDeg      (default: [0, 360]) deg
% - boulicMinStartSeparation     (default: 0.8) m
% - boulicRandomSeed             (default: random each run)
% - boulicThighHeight            (default: inferred from template) m
% - boulicSpineHeightRatio       (default: inferred from template)
% - boulicSwingClearance         (default: 0.14) m
% - boulicGroundZ                (default: 0.0) m

generated = false;

if getScalar(paraCfg, 'switchGenerateBoulicTargets', 0) ~= 1
    return
end

scenarioNameStr = paraCfg.inputScenarioName;
inputPath = fullfile(scenarioNameStr, 'Input');
assert(isfolder(inputPath), 'Input folder not found: %s', inputPath);

numTargets = max(0, round(getScalar(paraCfg, 'boulicNumberOfTargets', 0)));
if numTargets == 0
    warning(['switchGenerateBoulicTargets is enabled but ', ...
        'boulicNumberOfTargets is 0. Skipping generation.']);
    return
end

nTime = max(1, round(getScalar(paraCfg, 'numberOfTimeDivisions', 1)));
simTime = max(eps, getScalar(paraCfg, 'totalTimeDuration', 1));

templateScenario = getString(paraCfg, 'boulicTemplateScenario', ...
    'examples/SensingTwoTargets');
templateInputPath = fullfile(templateScenario, 'Input');
assert(isfolder(templateInputPath), ...
    'Boulic template Input folder not found: %s', templateInputPath);

calib = loadSkeletonCalibration(templateInputPath);

xRange = getRange(paraCfg, 'boulicStartXRange', [-2, 2]);
yRange = getRange(paraCfg, 'boulicStartYRange', [-2, 2]);
vRange = getRange(paraCfg, 'boulicVelocityRange', [0.6, 1.4]);
dirRangeDeg = getRange(paraCfg, 'boulicDirectionRangeDeg', [0, 360]);
dirRangeRad = deg2rad(dirRangeDeg);
minSep = max(0, getScalar(paraCfg, 'boulicMinStartSeparation', 0.8));
seed = getScalar(paraCfg, 'boulicRandomSeed', nan);

opts = struct();
opts.thighHeight = max(0.25, getScalar(paraCfg, 'boulicThighHeight', calib.thighHeight));
opts.spineHeightRatio = max(0.2, getScalar(paraCfg, ...
    'boulicSpineHeightRatio', calib.spineHeightRatio));
opts.swingClearance = max(0, getScalar(paraCfg, 'boulicSwingClearance', 0.14));
opts.groundZ = getScalar(paraCfg, 'boulicGroundZ', 0.0);
opts.calib = calib;

rngState = rng;
cleanupObj = onCleanup(@() rng(rngState)); %#ok<NASGU>
if isnan(seed)
    rng('shuffle');
else
    rng(seed);
end

starts = sampleStarts(numTargets, xRange, yRange, minSep);
velocities = randUniform(vRange, [numTargets, 1]);
directions = randUniform(dirRangeRad, [numTargets, 1]);

deleteMatchingFiles(inputPath, 'TargetBase*.dat');
deleteMatchingFiles(inputPath, 'TargetJoints*.dat');

for targetId = 1:numTargets
    [baseOut, jointsOut] = synthesizeBoulicTrajectory( ...
        nTime, simTime, starts(targetId,:), velocities(targetId), ...
        directions(targetId), opts);

    validateOutput(baseOut, jointsOut, targetId-1);

    baseFile = fullfile(inputPath, sprintf('TargetBase%d.dat', targetId-1));
    jointsFile = fullfile(inputPath, sprintf('TargetJoints%d.dat', targetId-1));
    writematrix(baseOut, baseFile, 'Delimiter', ',');
    writematrix(jointsOut, jointsFile, 'Delimiter', ',');
end

fprintf(['Generated %d Boulic (paper-based) target trajectories in %s ', ...
    '(template calibration: %s).\n'], numTargets, inputPath, templateScenario);

generated = true;
end

function [baseOut, jointsOut] = synthesizeBoulicTrajectory(nTime, simTime, startXY, speed, heading, opts)
%BOULIC TRAJECTORY SYNTHESIS
% Implements global/periodic terms based on Boulic et al. formulas and a
% procedural FK/IK skeleton consistent with qd-realization target format.

t = linspace(0, simTime, nTime).';

ht = opts.thighHeight;
rv = max(0, speed / max(ht, eps));
rvModel = min(rv, 2.3);

if rvModel < 1e-4
    rlc = 0;
    dc = inf;
    dsPct = 1.0;
else
    % Eq. (1): RLc = 1.346 * sqrt(RV)
    rlc = 1.346 * sqrt(rvModel);
    dc = rlc / max(rvModel, eps);
    ds = 0.752 * dc - 0.143;
    dsPct = clampScalar(ds / max(dc, eps), 0.45, 0.88);
end

if isinf(dc)
    phase = zeros(nTime, 1);
else
    phase = mod(t / max(dc, eps), 1.0);
end
phaseLeft = phase;
phaseRight = mod(phase + 0.5, 1.0);

% Appendix A: body translations
av = 0.015 * rvModel;
zOffset = (-av + av .* sin(2*pi*(2*phase - 0.35))) * ht;

if rvModel > 0.5
    al = -0.032;
    aa = -0.021;
else
    al = -0.128 * rvModel^2 + 0.128 * rvModel;
    aa = -0.084 * rvModel^2 + 0.084 * rvModel;
end
lateralOffset = al .* sin(2*pi*(phase - 0.1)) * ht;
phiA = 0.625 - dsPct;
leadLagOffset = aa .* sin(2*pi*(2*phase + 2*phiA)) * ht;

fwdTravel = speed * t + leadLagOffset;
baseZ = opts.groundZ + opts.spineHeightRatio * ht + zOffset;

fwd2 = [cos(heading), sin(heading)];
left2 = [-sin(heading), cos(heading)];

baseXY = startXY + fwdTravel .* fwd2 + lateralOffset .* left2;

baseOut = zeros(nTime, 6);
baseOut(:,1:2) = baseXY;
baseOut(:,3) = baseZ;

% Optional base rotations from Appendix B pelvis trajectories.
[pitchDeg, rollDeg, yawDeg] = pelvisAnglesDeg(rvModel, phase);
baseOut(:,4) = pitchDeg;
baseOut(:,5) = rollDeg;
baseOut(:,6) = yawDeg;

jointsOut = zeros(nTime, 48);

calib = opts.calib;

for i = 1:nTime
    p = phase(i);
    pL = phaseLeft(i);
    pR = phaseRight(i);

    % Pelvis rotations from paper Appendix B.
    [pitchD, rollD, yawD] = pelvisAnglesDeg(rvModel, p);
    rPelvis = rotz(deg2rad(yawD)) * roty(deg2rad(pitchD)) * rotx(deg2rad(rollD));

    local = zeros(16,3);

    % Axial skeleton anchors (torso/head/shoulders/hips)
    local(1,:) = applyR(rPelvis, calib.neutral(1,:));
    local(2,:) = applyR(rPelvis, calib.neutral(2,:));
    local(3,:) = applyR(rPelvis, calib.neutral(3,:));
    local(4,:) = applyR(rPelvis, calib.neutral(4,:));
    local(9,:) = applyR(rPelvis, calib.neutral(9,:));
    local(10,:) = applyR(rPelvis, calib.neutral(10,:));

    % Arms (Appendix D.2/D.3 inspired)
    as = deg2rad(9.88 * rvModel);
    shoulderRest = deg2rad(3);

    shoulderFlexRight = shoulderRest - as/2 - as*cos(2*pi*pL);
    shoulderFlexLeft = shoulderRest - as/2 - as*cos(2*pi*pR);

    elbowRest = deg2rad(20);
    elbowAmp = deg2rad(20 + 25 * min(rvModel, 2.3) / 2.3);
    elbowFlexRight = elbowRest + elbowAmp * 0.5 * (1 + sin(2*pi*(pL - 0.1)));
    elbowFlexLeft = elbowRest + elbowAmp * 0.5 * (1 + sin(2*pi*(pR - 0.1)));

    [elbowR, handR] = armFK(local(3,:), shoulderFlexRight, elbowFlexRight, ...
        +1, calib, rPelvis);
    [elbowL, handL] = armFK(local(4,:), shoulderFlexLeft, elbowFlexLeft, ...
        -1, calib, rPelvis);

    local(5,:) = elbowR;
    local(6,:) = elbowL;
    local(7,:) = handR;
    local(8,:) = handL;

    % Legs (procedural stance/swing + 2-link IK)
    stepSpan = max(0.12, 0.60 * rlc * ht);
    swingAmp = opts.swingClearance * (0.15 + 0.85 * min(rvModel, 2.3) / 2.3);

    [kneeR, ankleR, toeR] = legFK(local(9,:), pR, dsPct, stepSpan, swingAmp, ...
        +1, calib, rPelvis);
    [kneeL, ankleL, toeL] = legFK(local(10,:), pL, dsPct, stepSpan, swingAmp, ...
        -1, calib, rPelvis);

    local(11,:) = kneeR;
    local(12,:) = kneeL;
    local(13,:) = ankleR;
    local(14,:) = ankleL;
    local(15,:) = toeR;
    local(16,:) = toeL;

    % Local frame (x forward, y left) to world frame.
    for j = 1:16
        pos = local(j,:);
        xy = baseXY(i,:) + pos(1) * fwd2 + pos(2) * left2;
        z = baseZ(i) + pos(3);
        c = (j-1)*3 + (1:3);
        jointsOut(i,c) = [xy, z];
    end
end
end

function [kneeP, ankleP, toeP] = legFK(hipP, phaseLeg, dsPct, stepSpan, swingAmp, sideSign, calib, rPelvis)
%LEGFK Build knee/ankle/toe from stance/swing trajectory and 2-link IK.

if phaseLeg < dsPct
    u = phaseLeg / max(dsPct, eps);
    xWalk = +0.5*stepSpan - stepSpan*u;
    zLift = 0;
    toePitch = deg2rad(-8 + 16*u);
else
    u = (phaseLeg - dsPct) / max(1-dsPct, eps);
    xWalk = -0.5*stepSpan + stepSpan*u;
    zLift = swingAmp * sin(pi*u);
    toePitch = deg2rad(8 - 12*abs(2*u - 1));
end

hipLocal = invApplyR(rPelvis, hipP);
hipX = hipLocal(1);
hipZ = hipLocal(3);

ankleTargetX = calib.ankleXOffset + xWalk;
ankleTargetZ = calib.ankleZNeutral + zLift;

xRel = ankleTargetX - hipX;
zRel = ankleTargetZ - hipZ;

l1 = calib.thighLen;
l2 = calib.shankLen;

d = hypot(xRel, zRel);
d = clampScalar(d, abs(l1-l2)+1e-6, l1+l2-1e-6);

cosK = clampScalar((l1^2 + l2^2 - d^2)/(2*l1*l2), -1, 1);
kneeFlex = pi - acos(cosK);

alpha = atan2(xRel, -zRel);
beta = acos(clampScalar((l1^2 + d^2 - l2^2)/(2*l1*d), -1, 1));
hipFlex = alpha - beta;

kneeX = hipX + l1 * sin(hipFlex);
kneeZ = hipZ - l1 * cos(hipFlex);
ankleX = kneeX + l2 * sin(hipFlex + kneeFlex);
ankleZ = kneeZ - l2 * cos(hipFlex + kneeFlex);

toeX = ankleX + calib.footLen * cos(toePitch);
toeZ = ankleZ + calib.footLen * sin(toePitch);

hipY = hipLocal(2);
kneeY = hipY - sideSign * calib.kneeInward;
ankleY = hipY - sideSign * calib.ankleInward;
toeY = hipY - sideSign * calib.toeInward;

kneeP = applyR(rPelvis, [kneeX, kneeY, kneeZ]);
ankleP = applyR(rPelvis, [ankleX, ankleY, ankleZ]);
toeP = applyR(rPelvis, [toeX, toeY, toeZ]);
end

function [elbowP, handP] = armFK(shoulderP, shoulderFlex, elbowFlex, sideSign, calib, rPelvis)
%ARMFK Simple 2-link arm forward kinematics.

u = calib.upperArmLen;
f = calib.forearmLen;

vecUpper = [u*sin(shoulderFlex), sideSign*calib.armOutward, -u*cos(shoulderFlex)];
vecFore = [f*sin(shoulderFlex - elbowFlex), sideSign*calib.forearmOutward, ...
    -f*cos(shoulderFlex - elbowFlex)];

elbowP = shoulderP + applyR(rPelvis, vecUpper);
handP = elbowP + applyR(rPelvis, vecFore);
end

function [pitchDeg, rollDeg, yawDeg] = pelvisAnglesDeg(rv, phase)
%PELVISANGLESGDEG Appendix B pelvis trajectories.

a1 = (rv > 0.5) * 2 + (rv <= 0.5) * (-8*rv^2 + 8*rv);
pitchDeg = -a1 + a1 .* sin(2*pi*(2*phase - 0.1));

a2 = 1.66 * rv;
rollDeg = pelvisRollWave(a2, phase);

a3 = 4 * rv;
yawDeg = -a3 .* cos(2*pi*phase);
end

function rollDeg = pelvisRollWave(a2, phase)
% Piecewise waveform from paper Appendix B, "Rotation left/right".

if numel(phase) > 1
    rollDeg = zeros(size(phase));
    for i = 1:numel(phase)
        rollDeg(i) = pelvisRollWave(a2, phase(i));
    end
    return
end

p = phase;
if p < 0.15
    rollDeg = -a2 + a2 * cos(2*pi*(10/3) * p);
elseif p < 0.5
    rollDeg = -a2 - a2 * cos(2*pi*(10/7) * (p - 0.15));
elseif p < 0.65
    rollDeg = a2 - a2 * cos(2*pi*(10/3) * (p - 0.5));
else
    rollDeg = a2 + a2 * cos(2*pi*(10/7) * (p - 0.65));
end
end

function calib = loadSkeletonCalibration(templateInputPath)
%LOADSKELETONCALIBRATION Build neutral geometry and segment lengths.

calib = defaultCalibration();

ids = findTemplateIds(templateInputPath);
for k = 1:numel(ids)
    id = ids(k);
    baseFile = fullfile(templateInputPath, sprintf('TargetBase%d.dat', id));
    jointsFile = fullfile(templateInputPath, sprintf('TargetJoints%d.dat', id));

    if ~isfile(baseFile) || ~isfile(jointsFile)
        continue
    end

    base = readmatrix(baseFile);
    joints = readmatrix(jointsFile);

    if isempty(base) || isempty(joints)
        continue
    end
    if size(base,2) ~= 6 || size(joints,2) ~= 48
        continue
    end
    if size(base,1) ~= size(joints,1)
        continue
    end

    n = size(base,1);
    d = base(end,1:2) - base(1,1:2);
    if norm(d) < 1e-8
        fwd = [1, 0];
    else
        fwd = d / norm(d);
    end
    left = [-fwd(2), fwd(1)];

    local = zeros(n,16,3);
    for t = 1:n
        b = base(t,1:3);
        for j = 1:16
            c = (j-1)*3 + (1:3);
            p = joints(t,c) - b;
            local(t,j,1) = dot(p(1:2), fwd);
            local(t,j,2) = dot(p(1:2), left);
            local(t,j,3) = p(3);
        end
    end

    neutral = squeeze(mean(local,1));

    % Update calibration with template-derived values.
    calib.neutral = neutral;
    calib.upperArmLen = mean(distanceTrace(local, 3, 5));
    calib.forearmLen = mean(distanceTrace(local, 5, 7));
    calib.thighLen = mean(distanceTrace(local, 9, 11));
    calib.shankLen = mean(distanceTrace(local, 11, 13));
    calib.footLen = mean(distanceTrace(local, 13, 15));

    % Side offsets (symmetric averages).
    yHip = 0.5 * (abs(neutral(9,2)) + abs(neutral(10,2)));
    yKnee = 0.5 * (abs(neutral(11,2)) + abs(neutral(12,2)));
    yAnkle = 0.5 * (abs(neutral(13,2)) + abs(neutral(14,2)));
    yToe = 0.5 * (abs(neutral(15,2)) + abs(neutral(16,2)));

    calib.kneeInward = max(0, yHip - yKnee);
    calib.ankleInward = max(0, yHip - yAnkle);
    calib.toeInward = max(0, yHip - yToe);

    calib.ankleXOffset = 0.5 * (neutral(13,1) + neutral(14,1));
    calib.ankleZNeutral = 0.5 * (neutral(13,3) + neutral(14,3));

    calib.armOutward = 0.5 * (neutral(5,2) - neutral(3,2));
    calib.forearmOutward = 0.5 * (neutral(7,2) - neutral(5,2));

    calib.thighHeight = max(0.25, calib.thighLen + calib.shankLen);
    calib.spineHeightRatio = mean(base(:,3)) / max(calib.thighHeight, eps);

    return
end

warning(['No valid TargetBase/TargetJoints template pair found in %s. ', ...
    'Using default Boulic skeleton calibration.'], templateInputPath);
end

function ids = findTemplateIds(templateInputPath)
files = dir(fullfile(templateInputPath, 'TargetBase*.dat'));
ids = nan(numel(files), 1);
for i = 1:numel(files)
    tok = regexp(files(i).name, '^TargetBase(\d+)\.dat$', 'tokens', 'once');
    if ~isempty(tok)
        ids(i) = str2double(tok{1});
    end
end
ids = sort(ids(~isnan(ids)));
end

function d = distanceTrace(local, jA, jB)
% local: Time x Joint x XYZ, with 1-based joint ids in [1..16]
va = squeeze(local(:,jA,:));
vb = squeeze(local(:,jB,:));
d = sqrt(sum((va - vb).^2, 2));
end

function calib = defaultCalibration()
calib = struct();

% Joint order:
% 1 spine/chest, 2 head, 3 right shoulder, 4 left shoulder,
% 5 right elbow, 6 left elbow, 7 right hand, 8 left hand,
% 9 right hip, 10 left hip, 11 right knee, 12 left knee,
% 13 right ankle, 14 left ankle, 15 right toe, 16 left toe.
calib.neutral = [ ...
     0.018,  0.000,  0.518; ...
     0.026,  0.000,  0.752; ...
     0.018,  0.233,  0.518; ...
     0.018, -0.233,  0.518; ...
    -0.033,  0.230,  0.185; ...
    -0.032, -0.230,  0.225; ...
     0.048,  0.224, -0.030; ...
     0.048, -0.224, -0.030; ...
     0.000,  0.171,  0.000; ...
     0.000, -0.171,  0.000; ...
     0.058,  0.160, -0.421; ...
     0.058, -0.160, -0.421; ...
    -0.052,  0.151, -0.801; ...
    -0.052, -0.151, -0.801; ...
     0.152,  0.143, -0.885; ...
     0.152, -0.143, -0.885  ...
    ];

calib.upperArmLen = 0.337;
calib.forearmLen = 0.277;
calib.thighLen = 0.441;
calib.shankLen = 0.443;
calib.footLen = 0.257;

calib.kneeInward = 0.011;
calib.ankleInward = 0.020;
calib.toeInward = 0.028;

calib.ankleXOffset = -0.052;
calib.ankleZNeutral = -0.801;

calib.armOutward = -0.025;
calib.forearmOutward = -0.015;

calib.thighHeight = calib.thighLen + calib.shankLen;
calib.spineHeightRatio = 1.10;
end

function out = applyR(r, in)
out = (r * in(:)).';
end

function out = invApplyR(r, in)
out = (r.' * in(:)).';
end

function m = rotx(a)
ca = cos(a);
sa = sin(a);
m = [1, 0, 0; 0, ca, -sa; 0, sa, ca];
end

function m = roty(a)
ca = cos(a);
sa = sin(a);
m = [ca, 0, sa; 0, 1, 0; -sa, 0, ca];
end

function m = rotz(a)
ca = cos(a);
sa = sin(a);
m = [ca, -sa, 0; sa, ca, 0; 0, 0, 1];
end

function starts = sampleStarts(n, xRange, yRange, minSep)
starts = zeros(n,2);
maxAttempts = 1000;
for i = 1:n
    placed = false;
    for at = 1:maxAttempts
        cand = [randUniform(xRange, [1,1]), randUniform(yRange, [1,1])];
        if i == 1
            starts(i,:) = cand;
            placed = true;
            break
        end
        if all(vecnorm(starts(1:i-1,:) - cand, 2, 2) >= minSep)
            starts(i,:) = cand;
            placed = true;
            break
        end
    end
    if ~placed
        error(['Unable to sample non-overlapping start positions. ', ...
            'Increase range or reduce boulicMinStartSeparation.']);
    end
end
end

function validateOutput(baseOut, jointsOut, targetId)
assert(size(baseOut,2) == 6, 'TargetBase%d must have 6 columns.', targetId);
assert(size(jointsOut,2) == 48, 'TargetJoints%d must have 48 columns.', targetId);
assert(size(baseOut,1) == size(jointsOut,1), ...
    'Base/Joints row mismatch for target %d.', targetId);
assert(all(isfinite(baseOut(:))), 'Non-finite in TargetBase%d.', targetId);
assert(all(isfinite(jointsOut(:))), 'Non-finite in TargetJoints%d.', targetId);
end

function out = clampScalar(val, lo, hi)
out = min(max(val, lo), hi);
end

function r = randUniform(rangeVals, sz)
r = rangeVals(1) + (rangeVals(2)-rangeVals(1)) * rand(sz);
end

function deleteMatchingFiles(pathStr, pattern)
files = dir(fullfile(pathStr, pattern));
for i = 1:numel(files)
    delete(fullfile(files(i).folder, files(i).name));
end
end

function val = getScalar(s, fieldName, defaultVal)
if ~isfield(s, fieldName)
    val = defaultVal;
    return
end
raw = s.(fieldName);
if isnumeric(raw)
    val = double(raw);
else
    val = str2double(raw);
end
if isnan(val)
    val = defaultVal;
end
end

function txt = getString(s, fieldName, defaultTxt)
if ~isfield(s, fieldName)
    txt = defaultTxt;
    return
end
raw = s.(fieldName);
if isstring(raw)
    txt = char(raw);
elseif ischar(raw)
    txt = raw;
else
    txt = defaultTxt;
end
end

function rangeVals = getRange(s, fieldName, defaultVals)
if ~isfield(s, fieldName)
    rangeVals = defaultVals;
    return
end
raw = s.(fieldName);

if isnumeric(raw)
    vals = double(raw(:).');
elseif ischar(raw) || isstring(raw)
    vals = str2num(char(raw)); %#ok<ST2NM>
else
    vals = [];
end

if isempty(vals)
    rangeVals = defaultVals;
elseif isscalar(vals)
    rangeVals = [vals, vals];
else
    rangeVals = vals(1:2);
end

if rangeVals(1) > rangeVals(2)
    rangeVals = fliplr(rangeVals);
end
end
