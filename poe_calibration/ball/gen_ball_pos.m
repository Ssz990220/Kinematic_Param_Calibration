function T_balls = gen_ball_pos(n_balls)
%GEN_BALL_POS Summary of this function goes here


T_balls = zeros(4,4,n_balls);
theta_balls = (rand(1,n_balls) - 0.5) * 180;
r_balls = rand(1,n_balls) * 900 + 400;
z_balls = rand(1,n_balls) * 400 + 900;
for i = 1:n_balls
    R = rotz(theta_balls(i));
    x = r_balls(i) * cos(theta_balls(i) / 180);
    y = r_balls(i) * sin(theta_balls(i) / 180);
    
    T_balls(:,:,i) = [R,[x,y,z_balls(i)]';zeros(1,3),1];
end

end

