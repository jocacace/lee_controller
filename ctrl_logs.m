%%%
%   Log file for lee controller
%
%   File structure:
%       P[1:3]
%       RefP[4:6]
%       dP[7:9]
%       RefdP[10:12]
%       RefddP[13:15]
%       perror[16:18]
%       verror[19:21]
%       cmdP[22:24]
%
%%%

function ctrl_logs( filename )
    close all
    
    f = csvread(filename);
    
    currP = f(:, 1:3);
    refP = f(:, 4:6);
    dP = f(:,7:9);
    RefdP = f(:, 10:12);
    perror = f(:, 16:18);
    verror = f(:, 19:21);
    cmdP = f(:, 22:24);
    %RefddP = f(:, 13:15);
   
    figure(1);
    plot(currP(:,1))
    title('x');
    hold on
    plot(refP(:,1))
    plot(cmdP(:,1))
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
    plot(refP(:,3))
    plot(cmdP(:,3))
    legend('curr', 'ref', 'cmd')
    
    %figure(4);
    %plot(RefdP(:,1));   
    %hold on
    %plot(dP(:,1));   
    %title('vel x')
    %legend('ref', 'curr')
    
    %figure(5);
    %plot(RefdP(:,2));   
    %hold on
    %plot(dP(:,2));   
    %title('vel y')
    %legend('ref', 'curr')
    
    %figure(6);
    %plot(RefdP(:,3));   
    %hold on
    %plot(dP(:,3));   
    %title('vel z')
    %legend('ref', 'curr')
    figure(7);
    plot( perror(:,1) );   
    hold on
    plot( perror(:,2) );   
    plot( perror(:,3) );   
    title('ex')
    legend('x', 'y', 'z')

    figure(8);
    plot( verror(:,1) );   
    hold on
    plot( verror(:,2) );   
    plot( verror(:,3) );   
    title('dex')
    legend('x', 'y', 'z')

    
    

    
end