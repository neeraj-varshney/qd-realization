function generatedEnvironmentFile = generateOutdoorAmfFromLatLon( ...
    scenarioNameStr, latitude, longitude, queryRadiusMeters, varargin)
%GENERATEOUTDOORAMFFROMLATLON Build AMF environment from OSM buildings.
% Uses open-access Overpass API and writes a scenario-local AMF file.
%
% INPUTS:
% - scenarioNameStr: scenario root relative to src, e.g. examples/MyScenario
% - latitude: center latitude in decimal degrees
% - longitude: center longitude in decimal degrees
% - queryRadiusMeters: OSM query radius around center
%
% OUTPUT:
% - generatedEnvironmentFile: AMF filename written in Input/

p = inputParser;
addParameter(p, 'timeoutSec', 120);
addParameter(p, 'queryTimeoutSec', 60);
addParameter(p, 'retriesPerEndpoint', 2);
addParameter(p, 'useCachedOnFailure', 1);
addParameter(p, 'endpoints', { ...
    'https://overpass-api.de/api/interpreter', ...
    'https://overpass.kumi.systems/api/interpreter', ...
    'https://overpass.openstreetmap.ru/api/interpreter' ...
    });
parse(p, varargin{:});

timeoutSec = max(5, p.Results.timeoutSec);
queryTimeoutSec = max(5, p.Results.queryTimeoutSec);
retriesPerEndpoint = max(1, round(p.Results.retriesPerEndpoint));
useCachedOnFailure = (p.Results.useCachedOnFailure == 1);
endpoints = p.Results.endpoints;

inputPath = fullfile(scenarioNameStr, 'Input');
generatedEnvironmentFile = 'OutdoorGeneratedFromLatLon.amf';
generatedEnvironmentPath = fullfile(inputPath, generatedEnvironmentFile);

fprintf(['Generating outdoor AMF from open-access OSM API ', ...
    '(lat=%.7f, lon=%.7f, radius=%.1fm).\n'], ...
    latitude, longitude, queryRadiusMeters);

try
    buildingPolygons = fetchBuildingFootprints(latitude, longitude, ...
        queryRadiusMeters, timeoutSec, queryTimeoutSec, ...
        retriesPerEndpoint, endpoints);
catch err
    if useCachedOnFailure && isNonEmptyFile(generatedEnvironmentPath)
        warning(['OSM fetch failed (%s). Using existing generated AMF: %s'], ...
            err.message, generatedEnvironmentPath);
        return
    end
    rethrow(err)
end

if isempty(buildingPolygons)
    if useCachedOnFailure && isNonEmptyFile(generatedEnvironmentPath)
        warning(['No OSM buildings found for queried area. ', ...
            'Using existing generated AMF: %s'], generatedEnvironmentPath);
        return
    else
        error(['No OSM buildings found near latitude/longitude. ', ...
            'Increase outdoorQueryRadius or choose a denser area.']);
    end
end

meshObjects = buildMeshObjectsFromFootprints(buildingPolygons, queryRadiusMeters);
writeAmf(generatedEnvironmentPath, meshObjects);

numVertices = sum(arrayfun(@(o) size(o.vertices,1), meshObjects));
numTriangles = sum(arrayfun(@(o) size(o.triangles,1), meshObjects));
fprintf('Generated AMF environment: %s (%d vertices, %d triangles)\n', ...
    generatedEnvironmentPath, numVertices, numTriangles);
end


%% OSM fetch
function buildingPolygons = fetchBuildingFootprints(latitude, longitude, ...
    queryRadiusMeters, timeoutSec, queryTimeoutSec, retriesPerEndpoint, endpoints)
query = sprintf(['[out:json][timeout:%d];', ...
    '(way["building"](around:%.1f,%.8f,%.8f););', ...
    '(._;>;);', ...
    'out body;'], round(queryTimeoutSec), queryRadiusMeters, latitude, longitude);

response = [];
errMsgs = {};
options = weboptions('Timeout', timeoutSec, 'ContentType', 'json');
encodedQuery = char(java.net.URLEncoder.encode(query, 'UTF-8'));
for idx = 1:numel(endpoints)
    for attempt = 1:retriesPerEndpoint
        try
            url = sprintf('%s?data=%s', endpoints{idx}, encodedQuery);
            response = webread(url, options);
            break
        catch err
            errMsgs{end+1} = sprintf('[%s try %d/%d] %s', ... %#ok<AGROW>
                endpoints{idx}, attempt, retriesPerEndpoint, err.message);
            pause(min(2, 0.2 * attempt));
        end
    end
    if ~isempty(response)
        break
    end
end

if isempty(response) || ~isfield(response, 'elements')
    maxMsgs = min(numel(errMsgs), 4);
    if maxMsgs == 0
        msg = 'unknown failure';
    else
        msg = strjoin(errMsgs(1:maxMsgs), ' | ');
    end
    error('Unable to fetch OSM data from Overpass endpoints: %s', msg);
end

elements = response.elements;
nodeMap = containers.Map('KeyType', 'char', 'ValueType', 'any');

for i = 1:numel(elements)
    el = getElementAt(elements, i);
    if isfield(el, 'type') && strcmp(el.type, 'node') ...
            && isfield(el, 'id') && isfield(el, 'lat') && isfield(el, 'lon')
        key = num2str(el.id);
        nodeMap(key) = [el.lat, el.lon];
    end
end

buildingPolygons = struct('xy', {}, 'height', {});
for i = 1:numel(elements)
    el = getElementAt(elements, i);
    if ~(isfield(el, 'type') && strcmp(el.type, 'way') && ...
            isfield(el, 'nodes') && isfield(el, 'tags') && ...
            isfield(el.tags, 'building'))
        continue
    end
    wayNodes = el.nodes(:)';
    if numel(wayNodes) < 3
        continue
    end
    if wayNodes(1) == wayNodes(end)
        wayNodes(end) = [];
    end
    if numel(wayNodes) < 3
        continue
    end

    latLon = nan(numel(wayNodes), 2);
    valid = true;
    for n = 1:numel(wayNodes)
        key = num2str(wayNodes(n));
        if ~isKey(nodeMap, key)
            valid = false;
            break
        end
        latLon(n, :) = nodeMap(key);
    end
    if ~valid
        continue
    end

    xy = latLonToLocalMeters(latLon, latitude, longitude);
    xy = dedupeConsecutivePoints(xy);
    if size(xy, 1) < 3
        continue
    end
    if abs(polygonSignedArea(xy)) < 1e-6
        continue
    end

    buildingPolygons(end+1).xy = xy; %#ok<AGROW>
    buildingPolygons(end).height = buildingHeightFromTags(el.tags);
end
end


%% Mesh build
function meshObjects = buildMeshObjectsFromFootprints(buildingPolygons, queryRadiusMeters)
meshObjects = struct('name', {}, 'materialId', {}, 'vertices', {}, 'triangles', {});

for i = 1:numel(buildingPolygons)
    xy = buildingPolygons(i).xy;
    h = buildingPolygons(i).height;
    n = size(xy, 1);

    vertices = [[xy, zeros(n,1)]; [xy, h*ones(n,1)]];
    triangles = zeros(0,3);

    % Walls
    for k = 1:n
        k2 = mod(k, n) + 1;
        b1 = k;
        b2 = k2;
        t1 = n + k;
        t2 = n + k2;
        triangles = appendDoubleSided(triangles, [b1, b2, t2]); %#ok<AGROW>
        triangles = appendDoubleSided(triangles, [b1, t2, t1]); %#ok<AGROW>
    end

    % Roof and base
    roofTri = triangulateSimplePolygon(xy);
    for t = 1:size(roofTri, 1)
        bTri = roofTri(t, :);
        tTri = n + roofTri(t, :);
        triangles = appendDoubleSided(triangles, tTri); %#ok<AGROW>
        triangles = appendDoubleSided(triangles, bTri); %#ok<AGROW>
    end

    meshObjects(end+1).name = sprintf('building_%d', i); %#ok<AGROW>
    meshObjects(end).materialId = 1; % Buildings (CityBlock mapping)
    meshObjects(end).vertices = vertices;
    meshObjects(end).triangles = triangles;
end

% Ground object uses the same material naming as CityBlock.xml.
g = max(queryRadiusMeters*1.5, 50);
groundVertices = [-g,-g,0; g,-g,0; g,g,0; -g,g,0];
groundTriangles = zeros(0,3);
groundTriangles = appendDoubleSided(groundTriangles, [1,2,3]);
groundTriangles = appendDoubleSided(groundTriangles, [1,3,4]);

meshObjects(end+1).name = 'Ground'; %#ok<AGROW>
meshObjects(end).materialId = 2; % Ground (CityBlock mapping)
meshObjects(end).vertices = groundVertices;
meshObjects(end).triangles = groundTriangles;
end


%% AMF write
function writeAmf(amfPath, meshObjects)
fid = fopen(amfPath, 'w');
if fid < 0
    error('Cannot open AMF output file: %s', amfPath);
end
cleanupObj = onCleanup(@() fclose(fid)); %#ok<NASGU>

fprintf(fid, '<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n');
fprintf(fid, '<amf unit=\"meter\">\n');
for objId = 1:numel(meshObjects)
    obj = meshObjects(objId);
    fprintf(fid, '  <object id=\"%d\">\n', objId-1);
    fprintf(fid, '    <metadata type=\"name\">%s</metadata>\n', obj.name);
    fprintf(fid, '    <mesh>\n');
    fprintf(fid, '      <vertices>\n');
    for i = 1:size(obj.vertices,1)
        fprintf(fid, ['        <vertex><coordinates><x>%.6f</x><y>%.6f</y>', ...
            '<z>%.6f</z></coordinates></vertex>\n'], ...
            obj.vertices(i,1), obj.vertices(i,2), obj.vertices(i,3));
    end
    fprintf(fid, '      </vertices>\n');
    fprintf(fid, '      <volume materialid=\"%d\">\n', obj.materialId);
    for i = 1:size(obj.triangles,1)
        fprintf(fid, ['        <triangle><v1>%d</v1><v2>%d</v2>', ...
            '<v3>%d</v3></triangle>\n'], ...
            obj.triangles(i,1)-1, obj.triangles(i,2)-1, obj.triangles(i,3)-1);
    end
    fprintf(fid, '      </volume>\n');
    fprintf(fid, '    </mesh>\n');
    fprintf(fid, '  </object>\n');
end

% Match CityBlock material naming so materialLibraryCityBlock.csv works.
fprintf(fid, '  <material id=\"1\">\n');
fprintf(fid, '    <metadata type=\"name\">Buildings</metadata>\n');
fprintf(fid, '  </material>\n');
fprintf(fid, '  <material id=\"2\">\n');
fprintf(fid, '    <metadata type=\"name\">Ground</metadata>\n');
fprintf(fid, '  </material>\n');
fprintf(fid, '</amf>\n');
end


%% Geometry helpers
function xy = latLonToLocalMeters(latLon, lat0, lon0)
earthRadius = 6378137; % meters
x = earthRadius * deg2rad(latLon(:,2) - lon0) * cosd(lat0);
y = earthRadius * deg2rad(latLon(:,1) - lat0);
xy = [x, y];
end

function height = buildingHeightFromTags(tags)
height = parseNumericTag(tags, 'height');
if isnan(height)
    levels = parseNumericTag(tags, 'building:levels');
    if ~isnan(levels)
        height = levels * 3.0; % meters per level
    end
end
if isnan(height)
    height = 10.0; % default fallback
end
height = min(max(height, 3.0), 120.0);
end

function val = parseNumericTag(tags, tagName)
val = nan;
if isfield(tags, tagName)
    raw = tags.(tagName);
else
    altName = matlab.lang.makeValidName(tagName);
    if isfield(tags, altName)
        raw = tags.(altName);
    else
        return
    end
end
if isnumeric(raw)
    val = double(raw);
    return
end
if ~ischar(raw) && ~isstring(raw)
    return
end
txt = char(raw);
token = regexp(txt, '[-+]?\d+(\.\d+)?', 'match', 'once');
if ~isempty(token)
    val = str2double(token);
end
end

function out = dedupeConsecutivePoints(xy)
if isempty(xy)
    out = xy;
    return
end
keep = [true; sqrt(sum((diff(xy,1,1)).^2,2)) > 1e-6];
out = xy(keep, :);
if size(out,1) > 1 && norm(out(1,:) - out(end,:)) < 1e-6
    out(end,:) = [];
end
end

function area = polygonSignedArea(xy)
x = xy(:,1);
y = xy(:,2);
area = 0.5 * sum(x .* y([2:end,1]) - y .* x([2:end,1]));
end

function triangles = appendDoubleSided(triangles, tri)
triangles = [triangles; tri; tri([1,3,2])];
end

function tri = triangulateSimplePolygon(xy)
n = size(xy,1);
if n == 3
    tri = [1,2,3];
    return
end

idx = 1:n;
tri = zeros(0,3);
isCCW = polygonSignedArea(xy) > 0;
guard = 0;

while numel(idx) > 3
    guard = guard + 1;
    if guard > n*n
        % Fallback for hard polygons: simple fan triangulation.
        tri = [tri; [(idx(1)*ones(numel(idx)-2,1)), idx(2:end-1)', idx(3:end)']]; %#ok<AGROW>
        return
    end
    earFound = false;

    for i = 1:numel(idx)
        iPrev = idx(mod(i-2, numel(idx)) + 1);
        iCurr = idx(i);
        iNext = idx(mod(i, numel(idx)) + 1);

        a = xy(iPrev,:);
        b = xy(iCurr,:);
        c = xy(iNext,:);

        if ~isConvexCorner(a, b, c, isCCW)
            continue
        end

        anyInside = false;
        for j = 1:numel(idx)
            id = idx(j);
            if id == iPrev || id == iCurr || id == iNext
                continue
            end
            if pointInTriangle(xy(id,:), a, b, c)
                anyInside = true;
                break
            end
        end
        if anyInside
            continue
        end

        tri(end+1,:) = [iPrev, iCurr, iNext]; %#ok<AGROW>
        idx(i) = [];
        earFound = true;
        break
    end

    if ~earFound
        tri = [tri; [(idx(1)*ones(numel(idx)-2,1)), idx(2:end-1)', idx(3:end)']]; %#ok<AGROW>
        return
    end
end

tri(end+1,:) = idx; %#ok<AGROW>
end

function tf = isConvexCorner(a, b, c, isCCW)
ab = b - a;
bc = c - b;
crossZ = ab(1)*bc(2) - ab(2)*bc(1);
if isCCW
    tf = crossZ > 1e-9;
else
    tf = crossZ < -1e-9;
end
end

function tf = pointInTriangle(p, a, b, c)
v0 = c - a;
v1 = b - a;
v2 = p - a;

dot00 = dot(v0, v0);
dot01 = dot(v0, v1);
dot02 = dot(v0, v2);
dot11 = dot(v1, v1);
dot12 = dot(v1, v2);

den = dot00*dot11 - dot01*dot01;
if abs(den) < 1e-12
    tf = false;
    return
end
invDen = 1 / den;
u = (dot11*dot02 - dot01*dot12) * invDen;
v = (dot00*dot12 - dot01*dot02) * invDen;
tf = (u >= 0) && (v >= 0) && (u + v <= 1);
end

function el = getElementAt(elements, idx)
if iscell(elements)
    el = elements{idx};
else
    el = elements(idx);
end
end

function tf = isNonEmptyFile(pathStr)
tf = false;
if ~isfile(pathStr)
    return
end
d = dir(pathStr);
tf = ~isempty(d) && d.bytes > 0;
end
