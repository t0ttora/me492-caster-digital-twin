function export_m2_visuals(model)
root=fileparts(mfilename('fullpath'));folder=fullfile(root,'plots');if ~isfolder(folder),mkdir(folder);end
for row={model,'top_level.png';[model '/WheelGroundForces'],'contacts_subsystem.png'; ...
 [model '/BodyCasterInertiaDynamics'],'inertia_subsystem.png'}.'
 print(['-s' row{1}],'-dpng','-r240',fullfile(folder,row{2}));
end
[g,d,p]=m2_load_parameters();
states={'B_x','B_y','theta','phi','V_x','V_y','Omega','rho','omega_L','omega_R','omega_C','W_motor','D_loss'};
units={'m','m','rad','rad','m/s','m/s','rad/s','rad/s','rad/s','rad/s','rad/s','J','J'};
frames={'body COM, world X','body COM, world Y','body yaw in world, CCW', ...
 'caster angle relative to body','body COM world velocity X','body COM world velocity Y', ...
 'body yaw rate','relative caster swivel rate','left drive roll rate','right drive roll rate', ...
 'caster roll rate','integral of motor power','integral of dissipated power'};
fig=figure('Visible','off','Color','w','Position',[100 100 1350 750]);
ax=axes(fig,'Position',[0.03 0.08 0.94 0.78]);axis(ax,[0 1 0 14]);axis(ax,'off');
text(ax,0,13.4,'index     symbol          SI         meaning / frame','FontName','Consolas','FontSize',17,'FontWeight','bold');
for k=1:13
 text(ax,0,13-k,sprintf('x(%02d)     %-14s %-9s  %s',k,states{k},units{k},frames{k}), ...
 'FontName','Consolas','FontSize',15,'Interpreter','none');
end
annotation(fig,'textbox',[0.03 .89 .94 .08],'String','11 physical states + 2 energy integrals = 13 integrated states', ...
 'EdgeColor','none','FontSize',21,'FontWeight','bold');
annotation(fig,'textbox',[.03 .01 .94 .06],'String','psi = theta + phi; psiDot = Omega + rho. Wheel roll angles are cyclic; their rates are retained.', ...
 'EdgeColor','none','FontSize',14);exportgraphics(fig,fullfile(folder,'state_map.png'),'Resolution',240);close(fig);
labels={'body mass','caster lumped mass','body yaw inertia about B','caster yaw inertia about C','drive roll inertia','caster roll inertia', ...
 'body COM offset x from A','body COM offset y from A','body COM height (not used in planar dynamics)', ...
 'drive radius','drive track','caster radius','caster trail','pivot x from A','pivot y from A','gravity', ...
 'drive friction coefficient','caster friction coefficient','drive long slip damping','drive lateral slip damping', ...
 'caster long slip damping','caster lateral slip damping','drive rolling coefficient','caster rolling coefficient', ...
 'drive roll viscous damping','caster roll viscous damping','swivel viscous bearing','swivel Coulomb bearing', ...
 'rolling regularization rate','swivel regularization rate','caster bore effective radius','bore regularization rate'};
pu={'kg','kg','kg m^2','kg m^2','kg m^2','kg m^2','m','m','m','m','m','m','m','m','m','m/s^2', ...
 '1','1','N s/m','N s/m','N s/m','N s/m','1','1','N m s/rad','N m s/rad','N m s/rad','N m','rad/s','rad/s','m','rad/s'};
fig=figure('Visible','off','Color','w','Position',[100 100 1500 1150]);
ax=axes(fig,'Position',[.025 .05 .95 .85]);axis(ax,[0 1 0 34]);axis(ax,'off');
text(ax,0,33.2,'vector     value          SI              named parameter','FontName','Consolas','FontSize',16,'FontWeight','bold');
for k=1:32
 text(ax,0,33-k,sprintf('p(%02d)     %-13.6g %-15s %s',k,p(k),pu{k},labels{k}), ...
 'FontName','Consolas','FontSize',14,'Interpreter','none');
end
annotation(fig,'textbox',[.025 .93 .95 .05],'String','SI parameter transport map | values are provisional and SYNTHETIC', ...
 'EdgeColor','none','FontSize',20,'FontWeight','bold');
exportgraphics(fig,fullfile(folder,'parameter_map.png'),'Resolution',240);close(fig);
map=struct('state_symbols',{states},'state_units',{units},'state_frames',{frames}, ...
 'parameter_labels',{labels},'parameter_units',{pu},'parameter_values',p.');
fid=fopen(fullfile(root,'state_parameter_map.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(map,'PrettyPrint',true));fclose(fid);
coordinate_plot(g,d,fullfile(folder,'coordinate_frames.png'));
end
function coordinate_plot(g,d,path)
theta=25*pi/180;phi=45*pi/180;psi=theta+phi;R=[cos(theta) -sin(theta);sin(theta) cos(theta)];
A=[0;0];B=A+R*d.body_com_m(1:2);H=A+R*g.caster_pivot_m(1:2);C=H-g.caster_trail_m*[cos(psi);sin(psi)];
L=A+R*[0;g.drive_track_m/2];RR=A+R*[0;-g.drive_track_m/2];
fig=figure('Visible','off','Color','w','Position',[100 100 1350 850]);ax=axes(fig);hold(ax,'on');axis(ax,'equal');grid(ax,'on');
plot([L(1) RR(1) C(1) L(1)],[L(2) RR(2) C(2) L(2)],':','Color',[.6 .6 .6],'LineWidth',1.5);
plot([A(1) H(1) C(1)],[A(2) H(2) C(2)],'-','Color',[.18 .35 .48],'LineWidth',2);
points=[A B H C L RR];names={'A: axle ground origin','B: body COM','H: swivel pivot','C: caster axle center','left drive','right drive'};
for k=1:6,plot(points(1,k),points(2,k),'o','MarkerFaceColor',[.18 .35 .48],'MarkerSize',8); ...
 text(points(1,k)+.009,points(2,k)+.012,names{k},'FontSize',13);end
quiver(A(1),A(2),.13,0,0,'k','LineWidth',1.4);text(.14,0,'world +X','FontSize',13);
quiver(A(1),A(2),0,.12,0,'k','LineWidth',1.4);text(0,.14,'world +Y','FontSize',13);
quiver(A(1),A(2),.18*R(1,1),.18*R(2,1),0,'Color',[.18 .35 .48],'LineWidth',1.5);
text(.19*R(1,1),.19*R(2,1),'body +x / theta','FontSize',13);
quiver(C(1),C(2),.12*cos(psi),.12*sin(psi),0,'Color',[.2 .5 .35],'LineWidth',1.8);
text(C(1)+.12*cos(psi),C(2)+.12*sin(psi),'caster e(psi)','FontSize',13);
text(-.28,-.22,sprintf(['B = A + R(theta) c\nC = H - trail e(psi)\n' ...
 'psi = theta + phi; phi is BODY-RELATIVE\nWorld forces use +X/+Y; +yaw is CCW.\n' ...
 'Top view, SI metres. Schematic angle example, not a simulation result.']), ...
 'FontSize',14,'BackgroundColor','w');
xlim([-.33 .28]);ylim([-.25 .30]);xlabel('world X (m)');ylabel('world Y (m)');
title('Frames, COM and contact support triangle | synthetic geometry','FontSize',18);
exportgraphics(fig,path,'Resolution',240);close(fig);
end
