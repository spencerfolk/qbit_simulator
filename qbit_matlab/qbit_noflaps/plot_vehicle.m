function [body] = plot_vehicle(y,z,theta,l)
% Function for plotting the vehicle at position [y,z] with orientation theta

in2m = 0.0254;
l = 2;
chord = 5*in2m;  % Wing chord

% Create a triangle centered at y,z
pts = [y y-l/8 y+l/8 ; z+1*l z z]';

body = rotate(polyshape(pts),theta*180/pi,[y,z]);

% plot(rotate(body,theta,[y,z]),'FaceColor','red')
% hold on
% plot(y,z,'ko','linewidth',2)
% xlim([-10,10])
% ylim([-10,10])
% grid on

end