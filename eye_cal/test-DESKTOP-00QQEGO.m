link0 = Link('d',495,'a',175,'alpha',pi/2);
link1 = Link('d',0,'a',900,'alpha',0,'offset',pi/2);
link2 = Link('d',0,'a',175,'alpha',pi/2);
link3 = Link('d',0,'a',975.82,'alpha',pi/2, 'offset',10.33/180*pi);
% link4 = Link('d',0,'a',135,'alpha',-pi/2,'offset',0);
% link5 = Link('d',0,'a',0,'alpha',0);
robot = SerialLink([link0, link1, link2,link3]);
robot.plot([0,0,0,0]);