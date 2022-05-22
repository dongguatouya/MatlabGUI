function [xbox,ybox,prect]=getbox

 

if waitforbuttonpress %等待鼠标按下,按下鼠标返回False，按下按键返回True

    return

end

 

Hf = gcf;      % 得到鼠标点击的窗口
%Hf = gcbf;     
Ha = gca(Hf);  % 得到鼠标点击的坐标系

 

AxesPt = get(Ha,'CurrentPoint'); % 得到第一个鼠标点击数据点在坐标系中的位置

FigPt = get(Hf,'CurrentPoint');  % 坐标轴的CurrentPoint值为一个2*3的矩阵，第一行为离观察者最

%近的点的三维坐标，第2行为离观察者最远的点的三维坐标。默认的视角View = 90度的情况下，这两行的

%x和y坐标是相同的。一般情况下，只需要取pos第1行的前两个元素，第3个元素为z轴坐标，一般不用。

 

rbbox([FigPt 0 0],FigPt) % 用鼠标圈出矩形框，鼠标松开时返回

 

AxesPt = [AxesPt;get(Ha,'CurrentPoint')];%组合起始点和终止点

 

[Xlim,Ylim] = getn(Ha,'Xlim','Ylim');%得到一个对象的多个属性的句柄

 

%xbox = [min(AxesPt(:,1)) max(AxesPt(:,1))]; %得到所画矩形框两角点的横坐标
xbox = round([min(AxesPt(:,1)) max(AxesPt(:,1))]); %得到所画矩形框两角点的横坐标

xbox = [max(xbox(1),Xlim(1)) min(xbox(2),Xlim(2))]; %点不能在边界外
 

%ybox = [min(AxesPt(:,2)) max(AxesPt(:,2))];
ybox = [round(min(AxesPt(:,2))) round(max(AxesPt(:,2)))];

ybox = [max(ybox(1),Ylim(1)) min(ybox(2),Ylim(2))];
 

prect = [xbox(1) ybox(1) diff(xbox) diff(ybox)];

