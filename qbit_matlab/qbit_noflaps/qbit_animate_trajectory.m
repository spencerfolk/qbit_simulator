%%% Spencer Folk 2020
%%% The bulk of this code was written by Mathew Halm for purposes of
%%% plotting a quadrotor in MEAM 517. It was adapted for use in plotting
%%% the planar qbit.
function qbit_animate_trajectory(h, t, x_vec, y_d, z_d, Fdes, l, save)
% INPUTS
% h - FIGURE OBJECT
% t - [1xn] time vector
% x - [Mxn] state vector where M = 3 => y, z, theta
% y_d - [1xn] y desired position at time t
% z_d - [1xn] z desired position at time t
% Fdes - [2xn] desired force vector
% l - [1x1] length between rotors (arm length is l/2)
% save - boolean to determine whether we should save the gif (true) or not
% (false)

a = l*2;

dt = t(2) - t(1);
y = x_vec(1,:);
z = x_vec(2,:);
theta = x_vec(3,:) - pi/2;
stale = .01;
tic
i = 1;

filename = 'testAnimated.gif';

while i<=numel(t)
    start = toc;
    hold off;
    plot(y_d, z_d, 'k:','LineWidth',2);
    grid on
    hold on;
    
    
    % Plot path
    plot(y(1:i), z(1:i), 'r','LineWidth',2);
    
    % Plot the vehicle
%         plot([y(i) + a*cos(theta(i)), y(i) - a*cos(theta(i))],...
%              [z(i) + a*sin(theta(i)), z(i) - a*sin(theta(i))] , 'b','LineWidth',6);
    body = plot_vehicle(y(i),z(i),theta(i),l);
    
    plot(body,'FaceColor','blue')
    
    % Plot the force vector
    quiver(y(i),z(i),5*Fdes(1,i)/norm(Fdes(:,i)),5*Fdes(2,i)/norm(Fdes(:,i)),'m-','linewidth',2)
    
    % Plot body orientation
%     quiver(y(i),z(i), cos(theta(i))*2, sin(theta(i))*2,'b-','linewidth',2)
    
    B = 1.5;
    %     xlim([min(y_d) - B, max(y_d) + B]);
    %     ylim([min(z_d) - B, max(z_d) + B]);
    
    xlim([y(i)-10,y(i)+10])
    ylim([z(i)-10,z(i)+10])
    
    %     axis equal
    
    xlabel('y [m]');
    ylabel('z [m]');
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