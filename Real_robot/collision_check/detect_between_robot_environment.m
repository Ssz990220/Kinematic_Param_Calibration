function iscollide = detect_between_robot_environment(theta_all,is_visul,pos_of_sphere)
%% 第零步：检测关节角度是否符合基本范围
if check_theta(theta_all)==1
    iscollide=1;
    return;
end
%% 第一步：机械臂全局与局部正向运动学
w_all=[[0;0;1],[0;1;0],[0;1;0],[1;0;0],[0;1;0],[1;0;0]];
q_all=[[0;0;0],[0.175;0;0.495],[0.175;0;0.495+0.9],[0;0;0.495+0.9+0.175],[0.175+0.96;0;0.495+0.9+0.175],[0;0;0.495+0.9+0.175]];
% g0=[[0 0 1;0 1 0;-1 0 0],[1.27;0;1.57];0,0,0,1];  % 上述运动学参数已经与标准ABB4600_60_2.05构型一致
theta_all=theta_all/180*pi;
%% 第二步：依赖运动学的各连杆建模和静态的障碍物建模 （连杆3与4的部分可以简化为一个机械臂的正向运动学）
base_cly_center1=[0;0.02;0];
base_cly_center2=[0;0;0.68];
base_cly_radius=[0.4;0;0];base_cly=[base_cly_center1,base_cly_center2,base_cly_radius];       % 基底圆柱的3信息
g0=[eye(3),[0.175;-0.175;0.4];0 0 0 1];cly2_center1=exp_fkine(w_all(:,1),q_all(:,1),g0,theta_all(1));cly2_center1=cly2_center1(1:3,4);
g0=[eye(3),[0.175;-0.175;1.5];0 0 0 1];cly2_center2=exp_fkine(w_all(:,1:2),q_all(:,1:2),g0,theta_all(1:2));cly2_center2=cly2_center2(1:3,4);
cly2_radius=[0.15;0;0];cly2=[cly2_center1,cly2_center2,cly2_radius];          % 连杆二包络圆柱的3信息
g0=[eye(3),[0.21;-0.14;1.52];0 0 0 1];cly3_center1=exp_fkine(w_all(:,1:3),q_all(:,1:3),g0,theta_all(1:3));cly3_center1=cly3_center1(1:3,4);
g0=[eye(3),[0.21;0.25;1.52];0 0 0 1];cly3_center2=exp_fkine(w_all(:,1:3),q_all(:,1:3),g0,theta_all(1:3));cly3_center2=cly3_center2(1:3,4);
cly3_radius=[0.27;0;0];cly3=[cly3_center1,cly3_center2,cly3_radius];            % 连杆三包络圆柱的3信息
g0=[eye(3),[0.43;0;1.57];0 0 0 1];cly4_1_center1=exp_fkine(w_all(:,1:3),q_all(:,1:3),g0,theta_all(1:3));cly4_1_center1=cly4_1_center1(1:3,4);
g0=[eye(3),[0.55;0;1.57];0 0 0 1];cly4_1_center2=exp_fkine(w_all(:,1:3),q_all(:,1:3),g0,theta_all(1:3));cly4_1_center2=cly4_1_center2(1:3,4);
cly4_1_radius=[0.11;0;0];cly4_1=[cly4_1_center1,cly4_1_center2,cly4_1_radius];  
g0=[eye(3),[0.55;0;1.57];0 0 0 1];cly4_2_center1=exp_fkine(w_all(:,1:3),q_all(:,1:3),g0,theta_all(1:3));cly4_2_center1=cly4_2_center1(1:3,4);
g0=[eye(3),[0.87;0;1.57];0 0 0 1];cly4_2_center2=exp_fkine(w_all(:,1:3),q_all(:,1:3),g0,theta_all(1:3));cly4_2_center2=cly4_2_center2(1:3,4);
cly4_2_radius=[0.065;0;0];cly4_2=[cly4_2_center1,cly4_2_center2,cly4_2_radius];
g0=[eye(3),[0.87;0;1.57];0 0 0 1];cly4_3_center1=exp_fkine(w_all(:,1:3),q_all(:,1:3),g0,theta_all(1:3));cly4_3_center1=cly4_3_center1(1:3,4);
g0=[eye(3),[1.28;0;1.57];0 0 0 1];cly4_3_center2=exp_fkine(w_all(:,1:3),q_all(:,1:3),g0,theta_all(1:3));cly4_3_center2=cly4_3_center2(1:3,4);
cly4_3_radius=[0.14;0;0];cly4_3=[cly4_3_center1,cly4_3_center2,cly4_3_radius];      % 连杆4的3个分支包络圆柱的9信息(圆心选在轴线上，转轴4的角度无效)

l=0.190;w=0.045;h=0.160;        % l对应y，w对应z，h对应x
end_cuboid_ini_node=[[eye(3),[1.28+h;0+l/2;1.57+w/2];0 0 0 1],[eye(3),[1.28+h;0+l/2;1.57-w/2];0 0 0 1],[eye(3),[1.28+h;0-l/2;1.57+w/2];0 0 0 1],[eye(3),[1.28+h;0-l/2;1.57-w/2];0 0 0 1],[eye(3),[1.28;0+l/2;1.57+w/2];0 0 0 1],[eye(3),[1.28;0+l/2;1.57-w/2];0 0 0 1],[eye(3),[1.28;0-l/2;1.57+w/2];0 0 0 1],[eye(3),[1.28;0-l/2;1.57-w/2];0 0 0 1]];
end_cuboid_node=exp_fkine(w_all,q_all,end_cuboid_ini_node,theta_all);end_cuboid_node=end_cuboid_node(1:3,4:4:32);
end_cuboid_node=end_cuboid_node(:,[1 5 3 2]);
% gocator用更精细的长方体包络，得到运动后的8个顶点的位置
%% 障碍物建模：
% ob_cuboid1=[[1.6;-0.2;0.5],[0.6;-0.2;0.5],[1.6;-0.3;0.5],[1.6;-0.2;1]];
% ob_cuboid2=[[1.6;0.2;0.5],[0.6;0.2;0.5],[1.6;0.1;0.5],[1.6;0.2;1]]+[0;0.05;0];
% ob_cuboid3=[[1.6;0.6;0.5],[0.6;0.6;0.5],[1.6;0.5;0.5],[1.6;0.6;1]]+[0;0.17;0];% 3个间隔分布的长方体模拟有高度差异的工件
a=pos_of_sphere(1);b=pos_of_sphere(2);c=pos_of_sphere(3);
ob_cly1=[[a;b;0],[a;b;c],[150;0;0]]/1000;
ob_cuboid1=[[-300;1100;450],[-300;1100+700;450],[-300+1000;1100;450],[-300;1100;0]]/1000;
ob_cuboid2=[[-940;-2000;0],[-940+500;-2000;0],[-940;-2000+4000;0],[-940;-2000;400]]/1000;
ob_cuboid3=[[-900;-2000;0],[-900+3000;-2000;0],[-900;-2000+700;0],[-900;-2000;2000]]/1000;
ob_cuboid4=[[2000;-2000;0],[2000+100;-2000;0],[2000;-2000+4000;0],[2000;-2000;2000]]/1000;
ob_cuboid5=[[-900;-2000;2000],[-900+3000;-2000;2000],[-900;-2000+4000;2000],[-900;-2000;2000+100]]/1000;
ob_cuboid6=[[-900;-2000;-100],[-900+3000;-2000;-100],[-900;-2000+4000;-100],[-900;-2000;0]]/1000;
ob_cuboid7=[[-940;2000;0],[-940+3000;2000;0],[-940;2000+100;0],[-940;2000;2000]]/1000;
ob_cuboid8=[[-1040;-2000;0],[-1040+100;-2000;0],[-1040;-2000+4000;0],[-1040;-2000;2000]]/1000;
%%
% ob_environment=[ob_cuboid1,ob_cuboid2,ob_cuboid3];index_environment=[1;1;1];
ob_environment=[ob_cly1,ob_cuboid1,ob_cuboid2,ob_cuboid3,ob_cuboid4,ob_cuboid5,ob_cuboid6,ob_cuboid7,ob_cuboid8];index_environment=[0;1;1;1;1;1;1;1;1];
ob_robot=[cly2,cly3,cly4_1,cly4_2,cly4_3,end_cuboid_node];index_robot=[0;0;0;0;0;1];
%%
iscollide1=detect_between_cylinder(base_cly, cly4_3, is_visul);
if (iscollide1==1)
    iscollide=1;
%     fprintf('base_cly and cly4_3 collide');
    return
end
iscollide2=detect_between_cuboid_cylinder(end_cuboid_node, base_cly, is_visul);
if (iscollide2==1)
    iscollide=1;
%     fprintf('end_cuboid_node and cly4_3 collide');
    return
end
use1=0;
for i=1:length(index_robot)
    use2=0;
    for j=1:length(index_environment)
        a=index_robot(i);
        b=index_environment(j);
        if a==1&&b==1
            iscollide=detect_between_cuboid(ob_robot(:,(use1+1):(use1+4)), ob_environment(:,(use2+1):(use2+4)), is_visul);
            if (iscollide==1)
%                 [num2str(i),' ',num2str(j)]
                return
            end
            use2=use2+4;
        end
        if a==0&&b==1
            iscollide=detect_between_cuboid_cylinder(ob_environment(:,(use2+1):(use2+4)), ob_robot(:,(use1+1):(use1+3)), is_visul);
            if (iscollide==1)
%                 [num2str(i),' ',num2str(j)]
                return
            end
            use2=use2+4;
        end
        if a==1&&b==0
            iscollide=detect_between_cuboid_cylinder(ob_robot(:,(use1+1):(use1+4)), ob_environment(:,(use2+1):(use2+3)), is_visul);
            if (iscollide==1)
%                 [num2str(i),' ',num2str(j)]
                return
            end
            use2=use2+3;
        end
        if a==0&&b==0
            iscollide=detect_between_cylinder(ob_robot(:,(use1+1):(use1+3)), ob_environment(:,(use2+1):(use2+3)), is_visul);
            if (iscollide==1)
%                 [num2str(i),' ',num2str(j)]
                return
            end
            use2=use2+3;
        end
    end
    if a==1
        use1=use1+4;
    else
        use1=use1+3;
    end
end
iscollide=0;
end
function iscollide=detect_between_cuboid_cylinder(cuboid,cylinder_, is_visul)
if (is_visul==1)
    visulizaiton1(cuboid,cylinder_);
end
center1=cylinder_(:,1);center2=cylinder_(:,2);radius=cylinder_(1,3);
if ispoint_in_cuboid(center1,cuboid)==1      % 如果圆心1在长方体内部则直接GG
    iscollide = 1;
    return
end
v1=cuboid(:,2)-cuboid(:,1);v2=cuboid(:,3)-cuboid(:,1);v3=cuboid(:,4)-cuboid(:,1);
all_segments1=[cuboid(:,2),cuboid(:,1),cuboid(:,2)+v2,cuboid(:,1)+v2,cuboid(:,2)+v3,cuboid(:,1)+v3,cuboid(:,2)+v2+v3,cuboid(:,1)+v2+v3];
all_segments2=[cuboid(:,3),cuboid(:,1),cuboid(:,3)+v3,cuboid(:,1)+v3,cuboid(:,3)+v1,cuboid(:,1)+v1,cuboid(:,3)+v1+v3,cuboid(:,1)+v1+v3];
all_segments3=[cuboid(:,4),cuboid(:,1),cuboid(:,4)+v1,cuboid(:,1)+v1,cuboid(:,4)+v2,cuboid(:,1)+v2,cuboid(:,4)+v1+v2,cuboid(:,1)+v1+v2];
all_segments=[all_segments1,all_segments2,all_segments3];   % 原生的所有长方体边长

% 返璞归真 从解析式入手，等价于二次规划问题
% 可行思路一：求两个高度平面与12条边的交点，则8个顶点与新生交点必定包含了多面体的所有顶点
% 顶点投影后的包络极点构成多边形，判断圆心到多边形每条边的距离(首先圆心center1不会在多边形内部)
axis_line=center2-center1;cuboid_vertex=[cuboid,cuboid(:,2)+v2,cuboid(:,3)+v3,cuboid(:,3)+v3+v1,cuboid(:,2)+v3];
axis_=cha(axis_line,[0;0;1]);theta=acos(axis_line(3)/norm(axis_line));
if (norm(axis_)==0)
    rot_R=eye(3);
else
    rot_R = axis_theta_rot(axis_/norm(axis_),theta);
end
new_cylinder=rot_R*cylinder_(:,1:2);init = new_cylinder(:,1);
new_cuboid=rot_R*cuboid_vertex-new_cylinder(:,1);faces=[1 2 5 3;1 3 6 4;2 5 7 8;6 4 8 7;4 1 2 8;5 3 6 7];
new_cylinder=new_cylinder-new_cylinder(:,1);h=new_cylinder(3,2);
%%
if h<min(new_cuboid(3,:)) || 0>max(new_cuboid(3,:))    % 如果圆柱的整体高度低于长方体或者高于长方体，则无碰撞
    iscollide = 0;
    return
end
all_interpoints = cal_interpoint([0;h], rot_R*all_segments-init);  % 求取两个高度对长方体的交点(交点一定发生在边上)
point_all = new_cuboid;
point_all2 = [];
for i=1:size(point_all,2)
    if (point_all(3,i)>=0)&&(point_all(3,i)<=h)
        point_all2 = [point_all2 point_all(:,i)];
    end
end
point_all = [point_all2 all_interpoints];point_all=point_all(1:2,:);
if isempty(point_all)
    iscollide=0;
    return
end
%%
% figure(2);
k=convhull(point_all');    % 含高度截取的长方体投影的凸包
% patch(point_all(1,k),point_all(2,k),'b'); %检验投影的合理性
% hold on
% visulizaiton2([[0;0;0],[0;0;h],[radius;0;0]],'r');
% view([0 0 1])
% axis equal
%%
all_distance = zeros(1,(length(k)-1));
for i=1:(length(k)-1)
    all_distance(i) = distance([0;0],[0;0],point_all(:,k(i)),point_all(:,k(i+1)));
end
if (min(all_distance)<radius)
    iscollide = 1;
    return
else
    iscollide = 0;
    return
end
end
function interpoints = cal_interpoint(heights, segments)
interpoints = [];
for i=1:length(heights)
    if heights(i)>max(segments(end,:)) || heights(i)<min(segments(end,:))
        continue
    end
    for j=1:(size(segments,2)/2)
        p1=segments(:,2*j-1);p2=segments(:,2*j);
        h=heights(i);z1=segments(end,2*j-1);z2=segments(end,2*j);
        if (min([z1 z2])<h && h<max([z1 z2]))
            lamda = (h-z2)/(z1-z2);
            interpoints = [interpoints lamda*p1+(1-lamda)*p2];
        end
    end
end
end
function ispoint_in = ispoint_in_cuboid(point, cuboid)
ispoint_in = 0;
v1=cuboid(:,2)-cuboid(:,1);v2=cuboid(:,3)-cuboid(:,1);v3=cuboid(:,4)-cuboid(:,1);
lamda = [v1 v2 v3]\(point-cuboid(:,1));
if (sum(lamda>=0)+sum(lamda<=1))==6
    ispoint_in = 1;
end
end
function rotR = axis_theta_rot(axis,theta)
rotR = cos(theta)*eye(3)+(1-cos(theta))*(axis*axis')+sin(theta)*hight(axis);
end
function result = hight(r)
result = zeros(3);
result(1,2)=-r(3);
result(1,3)=r(2);
result(2,1)=r(3);
result(2,3)=-r(1);
result(3,1)=-r(2);
result(3,2)=r(1);
end
function visulizaiton1(cuboid,cylinder)
%绘制圆柱
%需要知道中轴线线段的位置，以及圆筒的半径
%中轴线两端点的坐标，圆柱的高度。
obstracle_R=cylinder(1,3);
obstracle_L_center=cylinder(:,1)';
obstracle_H_center=cylinder(:,2)';
 
 
%建立底面圆心所在的坐标系
Vector=obstracle_H_center-obstracle_L_center;
obstracle_hight=norm(Vector);
CZ=Vector/norm(Vector);
CZout=null(CZ);
CX=CZout(:,1);
CY=CZout(:,2);
CZ=CZ';
Trans=[[CX CY CZ obstracle_L_center'];0 0 0 1];
Lx=zeros(2,51);
Ly=Lx;
Lz=Lx;
 
for i=1:50
   Lx(1,i)=obstracle_R*cos(i*2*pi/50);
   Ly(1,i)=obstracle_R*sin(i*2*pi/50);
   Lz(2,i)=obstracle_hight;
   Lz(1,i)=0;
end
Lx(1,51)=Lx(1,1);
Ly(1,51)=Ly(1,1);
Lz(1,51)=0;
Lz(2,51)=obstracle_hight;
 
 
Lx(2,:)=Lx(1,:);
Ly(2,:)=Ly(1,:);
 
for i=1:51
    out=Trans*[Lx(1,i);Ly(1,i);Lz(1,i);1];
    Lx(1,i)=out(1);
    Ly(1,i)=out(2);
    Lz(1,i)=out(3);
    out=Trans*[Lx(2,i);Ly(2,i);Lz(2,i);1];
    Lx(2,i)=out(1);
    Ly(2,i)=out(2);
    Lz(2,i)=out(3);    
end
hold on; grid on;
v1=cuboid(:,2)-cuboid(:,1);v2=cuboid(:,3)-cuboid(:,1);v3=cuboid(:,4)-cuboid(:,1);
cuboid_vertex=[cuboid,cuboid(:,2)+v2,cuboid(:,3)+v3,cuboid(:,3)+v3+v1,cuboid(:,2)+v3];
faces=[1 2 5 3;1 3 6 4;2 5 7 8;6 4 8 7;4 1 2 8;5 3 6 7];
patch('Faces',faces,'Vertices',cuboid_vertex','Facecolor','b')
alpha(0.2)
axis equal;
% axis([0 160 -10 160 -10 120]);
view(-50,20);
surf(Lx,Ly,Lz,'FaceColor',[1,0,0]);
fill3(Lx(1,:),Ly(1,:),Lz(1,:),[1,0,0]);
fill3(Lx(2,:),Ly(2,:),Lz(2,:),[1,0,0]);
xlabel('X');
ylabel('Y');
zlabel('Z');
end

function iscollide=detect_between_cylinder(cylinder_1,cylinder_2, is_visul)
if is_visul==1
    visulizaiton2(cylinder_1,'r');hold on
    visulizaiton2(cylinder_2,'b');hold off
end
center1_1=cylinder_1(:,1);center1_2=cylinder_1(:,2);radius1=cylinder_1(1,3);
center2_1=cylinder_2(:,1);center2_2=cylinder_2(:,2);radius2=cylinder_2(1,3);
% ①判断轴线端的距离情况（柱面与柱面碰撞情况）
[dLS,index] = distance(center1_1',center1_2',center2_1',center2_2');
if index==1||index==0          % 不谈了，圆柱完整的碰撞太复杂，采用胶囊近似
    if (dLS-(radius1+radius2))<=0
        iscollide=1;
        return
    else
        iscollide=0;
        return
    end
end
% ②计算轴线与圆柱的碰撞情况
axis_line1=center1_2-center1_1;axis_line2=center2_2-center2_1;
axis_1=cha(axis_line1,[0;0;1]);theta1=acos(axis_line1(3)/norm(axis_line1));
axis_2=cha(axis_line2,[0;0;1]);theta2=acos(axis_line2(3)/norm(axis_line2));
if (norm(axis_1)==0)
    rot_R1=eye(3);
else
    rot_R1 = axis_theta_rot(axis_1/norm(axis_1),theta1);
end
if (norm(axis_2)==0)
    rot_R2=eye(3);
else
    rot_R2 = axis_theta_rot(axis_2/norm(axis_2),theta2);
end
new_cylinder1=rot_R1*cylinder_1(:,1:2);init1 = new_cylinder1(:,1);
new_cylinder2=rot_R2*cylinder_2(:,1:2);init2 = new_cylinder2(:,1);
h1=new_cylinder1(3,2)-new_cylinder1(3,1);h2=new_cylinder2(3,2)-new_cylinder2(3,1);
isinter1=detect_between_segment_cylinder(rot_R1*center2_1-init1, rot_R1*center2_2-init1, radius1, h1);% 2轴线与1底圆
isinter2=detect_between_segment_cylinder(rot_R2*center1_1-init2, rot_R2*center1_2-init2, radius2, h2);% 1轴线与2底圆
if (isinter1==1)||(isinter2==1)
    iscollide=1;
    return
end
isinter3=detect_between_cylinderupcircle_cylinderbottomcircle(cylinder_1,cylinder_2);% 底圆与底圆之间碰撞
if isinter3==1
    iscollide=1;
else
    iscollide=0;% 还有底圆与柱面碰撞的情况！！！
end
end
function isinter = detect_between_segment_cylinder(p1, p2, radius, h)
% 判断线段是否与圆柱底面圆相交
% 直接用线段所在直线与圆柱底面的交点，若在圆内，则相交；若在圆外，则分离
% 当直线与圆所在平面接近重合时，直接用圆心到线段距离判断，增强鲁棒性
if abs(p1(3))<0.00001&&abs(p2(3))<0.00001
    d1=distance([0;0],[0;0],p1(1:2),p2(1:2));
    if (d1<=radius)
        isinter=1;
        return
    else
        isinter=0;
        return
    end
end
if abs(p1(3)-h)<0.00001&&abs(p2(3)-h)<0.00001
    d2=distance([0;0],[0;0],p1(1:2),p2(1:2));
    if (d2<=radius)
        isinter=1;
        return
    else
        isinter=0;
        return
    end
end
lamda1=-p2(3)/(p1(3)-p2(3));
lamda2=(h-p2(3))/(p1(3)-p2(3));
true_segments=[];
if lamda1<=1&&lamda1>=0
    true_segments = [true_segments lamda1*p1+(1-lamda1)*p2];
end
if lamda2<=1&&lamda2>=0
    true_segments = [true_segments lamda2*p1+(1-lamda2)*p2];
end
if p1(3)<=h&&p1(3)>=0
    true_segments = [true_segments p1];
end
if p2(3)<=h&&p2(3)>=0
    true_segments = [true_segments p2];
end
if isempty(true_segments)
    isinter=0;
    return
else
    if size(true_segments,2)==1
        d=norm(true_segments(1:2,1));
        if d<=radius
            isinter=1;
        else
            isinter=0;
        end
    end
end
if size(true_segments,2)>=2
    d=distance([0;0],[0;0],true_segments(1:2,1),true_segments(1:2,2));
    if d<=radius
        isinter=1;
    else
        isinter=0;
    end
end
% d1=norm(lamda1*p1+(1-lamda1)*p2);
% d2=norm(lamda2*p1(1:2)+(1-lamda2)*p2(1:2));
% if (d1<=radius&&lamda1<=1&&lamda1>=0)||(d2<=radius&&lamda2<=1&&lamda2>=0)
%     isinter=1;
% else
%     isinter=0;
% end
end
function isinter = detect_between_cylinderupcircle_cylinderbottomcircle(cylinder_1,cylinder_2)
% ③计算底圆之间的距离情况(异面两圆相交，则必均通过两面交线；均通过两面交线，则比较两条共线线段是否有交集即可)
% 当两个平面接近重合时，直接用圆心距离进行判断(增强鲁棒性)
center1_1=cylinder_1(:,1);center1_2=cylinder_1(:,2);radius1=cylinder_1(1,3);
center2_1=cylinder_2(:,1);center2_2=cylinder_2(:,2);radius2=cylinder_2(1,3);
axis1=center1_2-center1_1;axis1=axis1/norm(axis1);
axis2=center2_2-center2_1;axis2=axis2/norm(axis2);
if norm(axis1-axis2)<0.001
    axis_=cha(axis1,[0;0;1]);theta=acos(axis1(3)/norm(axis1));
    if (norm(axis_)==0)
        rot_R=eye(3);
    else
        rot_R = axis_theta_rot(axis_/norm(axis_),theta);
    end
    new_cly1=rot_R*cylinder_1;new_cly1=new_cly1(:,1:2);
    new_cly2=rot_R*cylinder_2;new_cly2=new_cly2(:,1:2);limit=0.001;
    if (abs(new_cly1(3,1)-new_cly2(3,1))<=limit&&norm(center1_1-center2_1)<=(radius1+radius2))  ||  (abs(new_cly1(3,1)-new_cly2(3,2))<=limit&&norm(center1_1-center2_2)<=(radius1+radius2))  ||  (abs(new_cly1(3,2)-new_cly2(3,1))<=limit&&norm(center1_2-center2_1)<=(radius1+radius2))  ||  (abs(new_cly1(3,2)-new_cly2(3,2))<=limit&&norm(center1_2-center2_2)<=(radius1+radius2))
        isinter=1;% 这里0.01代表平面离得多近认为是同一个平面
        return
    else
        isinter=0;
        return
    end
end
line_vector=cha(axis1,axis2);
point1=[axis1';axis2';[1 1 1]]\[axis1'*center1_1;axis2'*center2_1;1];
point2=[axis1';axis2';[1 1 1]]\[axis1'*center1_1;axis2'*center2_2;1];
point3=[axis1';axis2';[1 1 1]]\[axis1'*center1_2;axis2'*center2_1;1];
point4=[axis1';axis2';[1 1 1]]\[axis1'*center1_2;axis2'*center2_2;1];
isinter1=detect_between_circle(center1_1,radius1,center2_1,radius2,point1,line_vector);
isinter2=detect_between_circle(center1_1,radius1,center2_2,radius2,point2,line_vector);
isinter3=detect_between_circle(center1_2,radius1,center2_1,radius2,point3,line_vector);
isinter4=detect_between_circle(center1_2,radius1,center2_2,radius2,point4,line_vector);
if (isinter1==0&&isinter2==0&&isinter3==0&&isinter4==0)
    isinter=0;
else
    isinter=1;
end
end
function isinter = detect_between_circle(center1,radius1,center2,radius2,point,line_vector)
% 求空间中两个圆是否相交，输入需要在外面计算出两个面公共交线
line_vector = line_vector/norm(line_vector);
lamda1=(center1-point)'*line_vector;
near_point1=point+lamda1*line_vector;d1=norm(near_point1-center1);
lamda2=(center2-point)'*line_vector;
near_point2=point+lamda2*line_vector;d2=norm(near_point2-center2);
if d1<=radius1&&d2<=radius2
    gap1=sqrt(radius1^2-d1^2);
    gap2=sqrt(radius2^2-d2^2);
    if ((lamda1-gap1)-(lamda2+gap2))*((lamda1+gap1)-(lamda2-gap2))<=0
        isinter=1;
        return
    end
end
isinter=0;
end
function visulizaiton2(cylinder2,color)
%绘制圆柱
%需要知道中轴线线段的位置，以及圆筒的半径
%中轴线两端点的坐标，圆柱的高度。
obstracle_R=cylinder2(1,3);
obstracle_L_center=cylinder2(:,1)';
obstracle_H_center=cylinder2(:,2)';
%建立底面圆心所在的坐标系
Vector=obstracle_H_center-obstracle_L_center;
obstracle_hight=norm(Vector);
CZ=Vector/norm(Vector);
CZout=null(CZ);
CX=CZout(:,1);
CY=CZout(:,2);
CZ=CZ';
Trans=[[CX CY CZ obstracle_L_center'];0 0 0 1];
Lx=zeros(2,51);
Ly=Lx;
Lz=Lx;
 
for i=1:50
   Lx(1,i)=obstracle_R*cos(i*2*pi/50);
   Ly(1,i)=obstracle_R*sin(i*2*pi/50);
   Lz(2,i)=obstracle_hight;
   Lz(1,i)=0;
end
Lx(1,51)=Lx(1,1);
Ly(1,51)=Ly(1,1);
Lz(1,51)=0;
Lz(2,51)=obstracle_hight;
 
 
Lx(2,:)=Lx(1,:);
Ly(2,:)=Ly(1,:);
 
for i=1:51
    out=Trans*[Lx(1,i);Ly(1,i);Lz(1,i);1];
    Lx(1,i)=out(1);
    Ly(1,i)=out(2);
    Lz(1,i)=out(3);
    out=Trans*[Lx(2,i);Ly(2,i);Lz(2,i);1];
    Lx(2,i)=out(1);
    Ly(2,i)=out(2);
    Lz(2,i)=out(3);    
end
hold on; grid on;
axis equal;
% axis([0 160 -10 160 -10 120]);
view(-50,20);
surf(Lx,Ly,Lz,'FaceColor',color);
fill3(Lx(1,:),Ly(1,:),Lz(1,:),color);
fill3(Lx(2,:),Ly(2,:),Lz(2,:),color);
xlabel('X');
ylabel('Y');
zlabel('Z');
end

function iscollide=detect_between_cuboid(cuboid,cuboid2, is_visul)
if is_visul==1
    visulizaiton3(cuboid,'r');hold on
    visulizaiton3(cuboid2,'b');hold off
end
v1=cuboid(:,2)-cuboid(:,1);v2=cuboid(:,3)-cuboid(:,1);v3=cuboid(:,4)-cuboid(:,1);
rot_R=inv([v1/norm(v1),v2/norm(v2),v3/norm(v3)]);
% rot_R=eye(3);
cuboid_vertex=[cuboid,cuboid(:,2)+v2,cuboid(:,3)+v3,cuboid(:,3)+v3+v1,cuboid(:,2)+v3];
new_cuboid_vertex=rot_R*cuboid_vertex;
max_x=max(new_cuboid_vertex(1,:));min_x=min(new_cuboid_vertex(1,:));
max_y=max(new_cuboid_vertex(2,:));min_y=min(new_cuboid_vertex(2,:));
max_z=max(new_cuboid_vertex(3,:));min_z=min(new_cuboid_vertex(3,:));
v1=cuboid2(:,2)-cuboid2(:,1);v2=cuboid2(:,3)-cuboid2(:,1);v3=cuboid2(:,4)-cuboid2(:,1);
all_segments1=[cuboid2(:,2),cuboid2(:,1),cuboid2(:,2)+v2,cuboid2(:,1)+v2,cuboid2(:,2)+v3,cuboid2(:,1)+v3,cuboid2(:,2)+v2+v3,cuboid2(:,1)+v2+v3];
all_segments2=[cuboid2(:,3),cuboid2(:,1),cuboid2(:,3)+v3,cuboid2(:,1)+v3,cuboid2(:,3)+v1,cuboid2(:,1)+v1,cuboid2(:,3)+v1+v3,cuboid2(:,1)+v1+v3];
all_segments3=[cuboid2(:,4),cuboid2(:,1),cuboid2(:,4)+v1,cuboid2(:,1)+v1,cuboid2(:,4)+v2,cuboid2(:,1)+v2,cuboid2(:,4)+v1+v2,cuboid2(:,1)+v1+v2];
all_segments=[all_segments1,all_segments2,all_segments3];   % 原生的所有长方体边长
cuboid_vertex2=[cuboid2,cuboid2(:,2)+v2,cuboid2(:,3)+v3,cuboid2(:,3)+v3+v1,cuboid2(:,2)+v3];
new_all_segments=rot_R*all_segments;
new_cuboid_vertex2=rot_R*cuboid_vertex2;
interpoints_z = cal_interpoint([min_z,max_z],new_all_segments);
point_all2 = [];
for i=1:size(new_cuboid_vertex2,2)
    if (new_cuboid_vertex2(3,i)>=min_z)&&(new_cuboid_vertex2(3,i)<=max_z)
        point_all2 = [point_all2 new_cuboid_vertex2(:,i)];
    end
end
point_all2 = [point_all2 interpoints_z];
if(isempty(point_all2))
    iscollide=0;
    return
end
point_all2=point_all2(1:2,:);
%%
% figure(2);
k=convhull(point_all2');
% patch(point_all2(1,k),point_all2(2,k),'r'); %检验投影的合理性
% hold on
% visulizaiton3(rot_R*cuboid,'b');
% view([0 0 1])
% axis equal
%%
new_all_segments2=zeros(2,(length(k)-1)*2);
for i=1:(length(k)-1)
    new_all_segments2(:,2*i-1)=point_all2(:,k(i));
    new_all_segments2(:,2*i)=point_all2(:,k(i+1));
end
interpoints_y = cal_interpoint([min_y,max_y],new_all_segments2);
cuboid_proj_vertex=point_all2(:,k);
point_all3 = [];
for i=1:size(cuboid_proj_vertex,2)
    if (cuboid_proj_vertex(2,i)>=min_y)&&(cuboid_proj_vertex(2,i)<=max_y)
        point_all3 = [point_all3 cuboid_proj_vertex(:,i)];
    end
end
point_all3 = [point_all3 interpoints_y];
if(isempty(point_all3))
    iscollide=0;
    return
end
point_all3=point_all3(1,:);
if (max(point_all3)-min_x)*(min(point_all3)-max_x)<=0
    iscollide=1;
else
    iscollide=0;
end
end
function visulizaiton3(cuboid,color)
v1=cuboid(:,2)-cuboid(:,1);v2=cuboid(:,3)-cuboid(:,1);v3=cuboid(:,4)-cuboid(:,1);
cuboid_vertex=[cuboid,cuboid(:,2)+v2,cuboid(:,3)+v3,cuboid(:,3)+v3+v1,cuboid(:,2)+v3];
faces=[1 2 5 3;1 3 6 4;2 5 7 8;6 4 8 7;4 1 2 8;5 3 6 7];
patch('Faces',faces,'Vertices',cuboid_vertex','Facecolor',color)
alpha(0.2)
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
view(-50,20);
end


function isoverrange = check_theta(theta)
isoverrange=0;
if theta(1)<-180||theta(1)>180||theta(2)>150||theta(2)<-90||theta(3)>75||theta(3)<-180||theta(4)<-180||theta(4)>180||theta(5)>120||theta(5)<-125||theta(6)>180||theta(6)<-180
    isoverrange=1;
    return
end
end