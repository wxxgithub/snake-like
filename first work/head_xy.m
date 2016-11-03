%头部结点偏转
function [x0,y0] = head_xy(x,y,theta)
        x_tmp = x + pi/4;
        y_tmp = sin(x_tmp+pi/2);
    if theta == 0
        x0 = x_tmp;
        y0 = y_tmp;
    else
        x0 = (x_tmp - x) * cos(theta) - (y_tmp - y) * sin(theta) + x;
        y0 = (y_tmp - y) * cos(theta) + (x_tmp - x) * sin(theta) + y;
    end
end

