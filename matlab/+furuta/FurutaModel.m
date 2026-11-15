classdef FurutaModel
    % Tangential pendulum, theta1=0 upright; slender rod, axial inertia neglected.
    methods (Static)
        function dx = derivative(x,u,p)
            required={'armInertia','mass','armLength','com','pendInertiaCom','armFriction','pendFriction','R','L','Ke','Kt','gear','efficiency','busV'};
            for k=required,assert(isfield(p,k{1})&&isfinite(p.(k{1})),'Furuta:Parameter','Missing physical parameter %s',k{1});end
            assert(all([p.armInertia p.mass p.armLength p.com p.pendInertiaCom p.R p.L p.gear p.efficiency p.busV]>0),'Furuta:Parameter','Physical parameters must be positive');
            a=x(2);w=x(3);v=x(4);i=x(5);J=p.pendInertiaCom+p.mass*p.com^2;
            c=p.mass*p.armLength*p.com;A=p.armInertia+p.mass*p.armLength^2;
            M=[A+J*sin(a)^2,c*cos(a);c*cos(a),J];assert(det(M)>0,'Furuta:Parameter','Non-positive inertia matrix');
            torque=p.efficiency*p.gear*p.Kt*i;
            rhs=[torque-p.armFriction*w-2*J*sin(a)*cos(a)*w*v+c*sin(a)*v^2;...
                p.mass*9.80665*p.com*sin(a)-p.pendFriction*v+J*sin(a)*cos(a)*w^2];
            acc=M\rhs;di=(u*p.busV-p.R*i-p.Ke*p.gear*w)/p.L;
            dx=[w;v;acc;di];
        end
        function X = simulate(t,u,p,x0)
            t=t(:);u=u(:);X=zeros(numel(t),5);X(1,:)=x0(:).';
            for k=2:numel(t)
                dt=t(k)-t(k-1);steps=max(1,ceil(dt/min(.001,p.L/p.R/4)));h=dt/steps;x=X(k-1,:).';
                assert(steps<=10000,'Furuta:Model','Electrical timescale requires excessive integration steps');
                for j=1:steps
                    a=furuta.FurutaModel.derivative(x,u(k-1),p);b=furuta.FurutaModel.derivative(x+h*a/2,u(k-1),p);
                    c=furuta.FurutaModel.derivative(x+h*b/2,u(k-1),p);d=furuta.FurutaModel.derivative(x+h*c,u(k-1),p);
                    x=x+h*(a+2*b+2*c+d)/6;
                end
                assert(all(isfinite(x)),'Furuta:Model','Simulation diverged');X(k,:)=x.';
            end
        end
    end
end
