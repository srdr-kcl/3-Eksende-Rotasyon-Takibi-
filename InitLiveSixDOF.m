function anim = InitLiveSixDOF(varargin)
p = inputParser;
addParameter(p, 'SamplePlotFreq', 5);
addParameter(p, 'Trail', 'Off');
addParameter(p, 'LimitRatio', 1);
addParameter(p, 'Position', []);
addParameter(p, 'FullScreen', false);
addParameter(p, 'View', [30 20]);
addParameter(p, 'AxisLength', 1);
addParameter(p, 'ShowArrowHead', 'on');
addParameter(p, 'Xlabel', 'X');
addParameter(p, 'Ylabel', 'Y');
addParameter(p, 'Zlabel', 'Z');
addParameter(p, 'Title', '6DOF Animation');
addParameter(p, 'ShowLegend', true);
parse(p, varargin{:});

trailMode = lower(p.Results.Trail);
axisLength = p.Results.AxisLength;
samplePlotFreq = p.Results.SamplePlotFreq;

fig = figure('Name', 'Rotation Tracking', 'NumberTitle', 'off');
if p.Results.FullScreen
    set(fig, 'Units', 'normalized', 'OuterPosition', [0 0 1 1]);
elseif ~isempty(p.Results.Position)
    set(fig, 'Position', p.Results.Position);
end

axis equal;
grid on;
view(p.Results.View);
xlabel(p.Results.Xlabel); ylabel(p.Results.Ylabel); zlabel(p.Results.Zlabel);
hold on;
orgHandle = plot3(0, 0, 0, 'ko', 'MarkerSize', 3, 'MarkerFaceColor', 'k');

quivXhandle = quiver3(0,0,0,0,0,0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
quivYhandle = quiver3(0,0,0,0,0,0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
quivZhandle = quiver3(0,0,0,0,0,0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
%legend([quivXhandle, quivYhandle, quivZhandle], {'X', 'Y', 'Z'}, 'Location', 'southwest');

if strcmp(trailMode, 'dotsonly')
    trailHandle = plot3(NaN, NaN, NaN, 'k.'); 
elseif strcmp(trailMode, 'all')
    trailHandle = plot3(NaN, NaN, NaN, 'k-');  
else
    trailHandle = [];
end

anim.fig = fig;
anim.orgHandle = orgHandle;
anim.quivXhandle = quivXhandle;
anim.quivYhandle = quivYhandle;
anim.quivZhandle = quivZhandle;
anim.axisLength = axisLength;
anim.trailHandle = trailHandle;
anim.trailData = [];       
anim.samplePlotFreq = samplePlotFreq;
anim.trailMode = trailMode;

end