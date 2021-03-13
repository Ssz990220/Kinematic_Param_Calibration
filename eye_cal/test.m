link0 = Link('d',495,'a',175,'alpha',pi/2);
link1 = Link('d',0,'a',900,'alpha',0,'offset',pi/2);
link2 = Link('d',0,'a',200,'alpha',0);
link3 = Link('d',0,'a',760,'alpha',
robot = SerialLink([link0,link1,link2]);
robot.plot([0,0,0])