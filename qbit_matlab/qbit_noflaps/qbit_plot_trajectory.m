%%% Spencer Folk 2020
%%% The bulk of this code was written by Mathew Halm for purposes of
%%% plotting a quadrotor in MEAM 517. It was adapted for use in plotting
%%% the planar qbit.
function qbit_plot_trajectory(h, t, x_vec, x_d, z_d, Fdes, l, save)
% INPUTS
% h - FIGURE OBJECT
% t - [1xn] time vector
% x - [Mxn] state vector where M = 3 => x, z, phi 
% x_d - [1xn] x desired position at time t
% z_d - [1xn] x desired position at time t
% Fdes - [2xn] desired force vector
% l - [1x1] length between rotors (arm length is l/2)
% save - boolean to determine whether we should save the gif (true) or not
% (false)

a = l*2;

dt = t(2) - t(1);
x = x_vec(1,:);
z = x_vec(2,:);
phi = x_vec(3,:) - pi/2;
stale = .01;
tic
i = 1;

filename = 'testAnimated.gif';

while i<=numel(t)
    start = toc;
    hold off;
    plot(x_d, z_d, 'k:','LineWidth',2);
    grid on
    hold on;
    
    
    plot(x(1:i), z(1:i), 'r','LineWidth',2);
    
    plot([x(i) + a*cos(phi(i)), x(i) - a*cos(phi(i))],...
         [z(i) + a*sin(phi(i)), z(i) - a*sin(phi(i))] , 'b','LineWidth',6);
    quiver(x(i),z(i),5*Fdes(1,i)/norm(Fdes(:,i)),5*Fdes(2,i)/norm(Fdes(:,i)),'m-','linewidth',2)
    quiver(x(i),z(i), cos(phi(i))*2, sin(phi(i))*2,'b-','linewidth',2)

    B = 1.5;
    xlim([min(x_d) - B, max(x_d) + B]);
    ylim([min(z_d) - B, max(z_d) + B]);
    
    axis equal

    xlabel('x');
    ylabel('z');
    titl = sprintf('QBiT Trajectory, $t =  %.2f $',t(i));
    title(titl,'Interpreter','latex');

    
    compu = toc - start;
    stale_i = max(stale,compu*2);
    next_i = find(t >= start + stale_i);
    
    if save == true
    % Create gif
    frame = getframe(h);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im,256);
    
    %Write to PNG
    newname = sprintf('Image_%d.png',i);
    imwrite(im,newname,'PNG');
    
%     % Write to the GIF File 
%     if i == 1 
%         imwrite(imind,cm,filename,'gif', 'Loopcount',inf); 
%     else 
%         imwrite(imind,cm,filename,'gif','WriteMode','append'); 
%     end 
    end
    
%     if numel(next_i) < 1
%         if i < numel(t)
%             i = numel(t);
%         else
%             break;
%         end
%     else
%         i = next_i(1);
%     end

    i = i + 10;
    pause(1e-3);

end