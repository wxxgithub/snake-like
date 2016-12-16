function angle=decision_Fuzzy(fisMat,three_dis)

global xy_nodes;
global coordinate_nodes;
global xy_nodes_world;
%尺度变换因子
k=0.1;
l=20;;
    forward_dis = k*three_dis(1,1);
    left_dis = k*three_dis(2,1);
    right_dis = k*three_dis(3,1);
          angle = evalfis([forward_dis left_dis right_dis],fisMat);
          angle = angle*l;
 
    %if detect_t <= 0 && angle~=0
       %worm_turn(angle);


    %end

end