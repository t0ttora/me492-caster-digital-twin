function export_m2_results()
% Two readable panels per native MATLAB figure for the Turkish tutorial.
root=fileparts(mfilename('fullpath'));a=read_case('aligned_forward');m=read_case('misaligned_forward');off=read_case('caster_contact_disabled');
fig=newfigure();tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
nexttile;plot(a.x_m,a.y_m,m.x_m,m.y_m,off.x_m,off.y_m,'LineWidth',2.0);grid on;axis equal;
xlabel('axle origin A: world X (m)');ylabel('world Y (m)');title('Same left/right torques, different paths');
legend('aligned caster','45 deg caster','caster tangential effects off','Location','best','FontSize',13);
nexttile;plot(m.time_s,rad2deg([a.yaw_rad m.yaw_rad off.yaw_rad]),'LineWidth',2.0);grid on;
xlabel('time (s)');ylabel('body yaw theta (deg)');title('Ground-contact feedback changes yaw');
legend('aligned caster','45 deg caster','tangential effects off','Location','best','FontSize',13);
sgtitle('ME492 M2 v1 | open-loop torque input | SYNTHETIC parameters','FontSize',21);
format(fig);exportgraphics(fig,fullfile(root,'plots','trajectory_and_yaw.png'),'Resolution',180);close(fig);
coast=read_case('passive_coast');fine=read_case('misaligned_fine');finer=read_case('misaligned_finer');
fig=newfigure();tiledlayout(fig,1,2,'TileSpacing','compact','Padding','compact');
nexttile;plot(coast.time_s,[coast.kinetic_energy_J coast.dissipated_energy_J coast.kinetic_energy_J+coast.dissipated_energy_J],'LineWidth',2);grid on;
xlabel('time (s)');ylabel('energy (J)');title('Zero input: kinetic energy becomes loss');
legend('kinetic T','dissipated D','T + D','Location','best','FontSize',13);
nexttile;semilogy(m.time_s,max(abs(m.yaw_rad-fine.yaw_rad),1e-16),m.time_s,max(abs(fine.yaw_rad-finer.yaw_rad),1e-16),'LineWidth',2);grid on;
xlabel('time (s)');ylabel('absolute yaw difference (rad)');title('Step refinement: h, h/2, h/4');
legend('h versus h/2','h/2 versus h/4','Location','best','FontSize',13);
sgtitle('Numerical consistency checks | energy and step size | no robot validation','FontSize',21);
format(fig);exportgraphics(fig,fullfile(root,'plots','energy_and_convergence.png'),'Resolution',180);close(fig);
 function t=read_case(name),t=readtable(fullfile(root,'results',[name '_states.csv']));end
end
function f=newfigure()
f=figure('Visible','off','Color','w','Position',[100 100 1450 720]);if isprop(f,'Theme'),f.Theme='light';end
end
function format(f)
axesList=findall(f,'Type','axes');set(axesList,'FontSize',16,'FontName','Arial');
end
