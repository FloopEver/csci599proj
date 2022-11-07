% A* algorithm
clc; clear; close all;
obstacleMatrix = csvread("./data_csv/obstacleMatrix.csv");
RobstacleMatrix = csvread("./data_csv/RobstacleMatrix.csv")';
cylinderMatrix = csvread("./data_csv/cylinderMatrix.csv");
cylinderRMatrix = csvread("./data_csv/cylinderRMatrix.csv")';
cylinderHMatrix = csvread("./data_csv/cylinderHMatrix.csv")';
start = csvread("./data_csv/start.csv")';
goal = csvread("./data_csv/goal.csv")';
[numberOfSphere, ~] = size(obstacleMatrix);
[numberOfCylinder, ~] = size(cylinderMatrix);
Alldirec = [[1,0,0];[0,1,0];[0,0,1];[-1,0,0];[0,-1,0];[0,0,-1];...
            [1,1,0];[1,0,1];[0,1,1];[-1,-1,0];[-1,0,-1];[0,-1,-1];...
            [1,-1,0];[-1,1,0];[1,0,-1];[-1,0,1];[0,1,-1];[0,-1,1];...
            [1,1,1];[-1,-1,-1];[1,-1,-1];[-1,1,-1];[-1,-1,1];[1,1,-1];...
            [1,-1,1];[-1,1,1]];
threshold = 0.7;
stop = threshold*1.5;
g = [start, 0; goal, inf]; % 每一行前三个数为点坐标，第四个数为路径耗散
Path = [];
Parent = [];
Open = [start, g(findIndex(g,start),4) + getDist(start,goal)];
%% 绘制障碍环境
figure(1)
[n,~] = size(obstacleMatrix);
for i = 1:n   %绘制静态球障碍物
    [x,y,z] = sphere();
    surfc(RobstacleMatrix(i)*x+obstacleMatrix(i,1),...
        RobstacleMatrix(i)*y+obstacleMatrix(i,2),...
        RobstacleMatrix(i)*z+obstacleMatrix(i,3));
    hold on;
end

[n,~] = size(cylinderMatrix);
for i = 1:n   %绘制圆柱体障碍物
    [x,y,z] = cylinder(cylinderRMatrix(i));
    z(2,:) = cylinderHMatrix(i);
    surfc(x + cylinderMatrix(i,1),y + cylinderMatrix(i,2),...
        z,'FaceColor','interp');
    hold on;
end

bar1 = scatter3(start(1),start(2),start(3),80,"cyan",'filled','o');hold on
bar2 = scatter3(goal(1),goal(2),goal(3),80,"magenta",'filled',"o");
axis equal
set(gcf,'unit','centimeters','position',[30 10 20 15]);
%% 主循环
while ~isempty(Open)
    [xi, index] = findMin(Open);
    Open(index,:) = [];
    if getDist(xi, goal) < stop
        break;
    end
    children = getChildren(xi, Alldirec, threshold, obstacleMatrix, RobstacleMatrix,...
                           cylinderMatrix, cylinderRMatrix, cylinderHMatrix);
    scatter3(children(:,1),children(:,2),children(:,3),10,'filled','o');
    drawnow;
    [n,~] = size(children);
    for i = 1:n
        child = children(i,:);
        if findIndex(g, child) == 0   % child不在g
            g = [g; child, inf];
        end
        a = g(findIndex(g, xi),4) + getDist(xi,child);
        if a < g(findIndex(g, child),4)
            g(findIndex(g, child),4) = a;
            Parent = setParent(Parent, child,xi);
            Open = setOpen(Open, child, a, goal);
        end
    end  
end
lastPoint = xi;
%% 回溯轨迹
x = lastPoint;
Path = x;
[n,~] = size(Parent);
while any(x ~= start)
    for i = 1:n
        if Parent(i,1:3) == x
            Path = [Parent(i,4:6); Path];
            break;
        end
    end
    x = Parent(i,4:6);
end
plot3([Path(:,1);goal(1)],[Path(:,2);goal(2)],[Path(:,3);goal(3)],'LineWidth',3,'color','r');
%% 计算轨迹距离
pathLength = 0;
[n,~] = size(Path);
for i = 1:n-1
    pathLength = pathLength + getDist(Path(i,:),Path(i+1,:));
end
pathLength = pathLength + getDist(Path(end,:),goal);
fprintf('路径的长度为:%f',pathLength);
%% 函数
function children = getChildren(pos, Alldirec, step,circleCenter,circleR, cylinderCenter,cylinderR, cylinderH)
allchild = [];
[n,~] = size(Alldirec);
for i = 1:n
    direc = Alldirec(i,:);
    child = pos + direc * step;
    if ~checkCol(child, circleCenter,circleR, cylinderCenter,cylinderR, cylinderH)
        continue;
    end
    allchild = [allchild; child];
end
children = allchild;
end

function flag = checkCol(pos, circleCenter,circleR, cylinderCenter,cylinderR, cylinderH)
[numberOfSphere, ~] = size(circleCenter);
[numberOfCylinder, ~] = size(cylinderCenter);
flag = true;
for i = 1:numberOfSphere
    if getDist(pos, circleCenter(i,:)) <= circleR(i)
        flag = false;
        break;
    end
end
for i = 1:numberOfCylinder
    if getDist(pos(1:2), cylinderCenter(i,:)) <= cylinderR(i) && pos(3) <= cylinderH(i)
        flag = false;
        break;
    end
end
if pos(3) <= 0 flag = false; end
end

function Par = setParent(Parent, xj, xi)
[n,~] = size(Parent);
if n == 0
    Par = [xj, xi];
else
    for i = 1:n
        if Parent(i,1:3) == xj
            Parent(i,4:6) = xi;
            Par = Parent;
            break;
        end
        if i == n
            Par = [Parent; xj, xi];
        end
    end
end
end

function Ope = setOpen(Open, child, a, goal)
[n,~] = size(Open);
if n == 0
    Ope = [child, a + getDist(child, goal)];
else
    for i = 1:n
        if Open(i,1:3) == child
            Open(i,4) = a + getDist(child, goal);
            Ope = Open;
        end
        if i == n
            Ope = [Open; child, a + getDist(child, goal)];
        end
    end
end
end

function h = heuristic(pos, goal)
h = max([abs(goal(1) - pos(1)),abs(goal(2) - pos(2)),abs(goal(3) - pos(3))]);
end

function index = findIndex(g, pos)
[n,~] = size(g);
index = 0;    % 表示没有找到索引
for i = 1:n
    if g(i,1:3) == pos
        index = i;   % 索引为i
        break;
    end
end
end

function d = getDist(x,y)
d = sqrt(sum((x - y).^2));
end

function [pos, index] = findMin(Open)
[~,index] = min(Open(:,4));
pos = Open(index,1:3);
end
