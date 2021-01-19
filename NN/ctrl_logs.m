%%%
%   Log file for lee controller
%
%   File structure:
%       P[1:3]
%       RefP[4:6]
%       CmdP[7:9]
%       yaw[10]
%       refYaw[11]
%       cmdYaw[12]
%       ExtForce[13:15]
%       ExtMoment[16:18]
%
%%%

function ctrl_logs( filename )

    f = csvread(filename);
    currP(:,1:3) = f(:, 1:3);
    refP = f(:, 4:6);
    cmdP = f(:,7:9);
    extForce = f(:, 13:15);
    extMom = f(:, 16:18);
   
    figure(1);
    plot(currP(:,1))
    title('x');
    hold on
    plot(cmdP(:,1))
    plot(refP(:,1))
    legend('curr', 'ref', 'cmd')
    
    
    figure(2);
    plot(currP(:,2))
    title('y');
    hold on
    plot(cmdP(:,2))
    plot(refP(:,2))
    legend('curr', 'ref', 'cmd')
    
    
    figure(3);
    plot(currP(:,3))
    title('z');
    hold on
    plot(cmdP(:,3))
    plot(refP(:,3))
    legend('curr', 'ref', 'cmd')
    
    figure(4);
    plot( extForce );
    title('extForce')
    legend('fx', 'fy', 'fz')
    
    
    figure(5);
    plot( extMom );
    title('extMom')
    legend('Mx', 'My', 'Mz')
    
    
end