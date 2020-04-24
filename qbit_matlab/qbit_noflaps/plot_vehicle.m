function [body] = plot_vehicle(x,z,phi,l)
% Function for plotting the vehicle at position [x,z] with orientation phi

in2m = 0.0254;
l = 2;
chord = 5*in2m;  % Wing chord

% Create a triangle centered at x,z
pts = [x x-l/8 x+l/8 ; z+1*l z z]';

body = rotate(polyshape(pts),phi*180/pi,[x,z]);

% plot(rotate(body,phi,[x,z]),'FaceColor','red')
% hold on
% plot(x,z,'ko','linewidth',2)
% xlim([-10,10])
% ylim([-10,10])
% grid on

end