window.onload = function()
{
    // state = [u,v,r,psi,xpos,ypos,delta];
    state = [0,0,0,0,1000,5000,0]
    ui = 0;

    // console.log(dX([1,1,1,1,1,1,1],1))
    
    canvas = document.getElementById("canvas");
    context = canvas.getContext("2d");
    ship = new Image();
    ship.src="myimage.png";

    window.addEventListener("keydown", keypress_handler, false);
    window.addEventListener("keyup", keyup_handler, false);

    var integrate = setInterval(advanceOneStep, 5);
    var plot = setInterval(draw, 50);
};

function dX(x, ui) {
    // returns the time derivates of the state vector
    
    // INPUTS: 
    // x = [u v r psi xpos ypos delta]
    // u      = perturbed surge velocity [m/s]
    // v      = perturbed sway velocity [m/s]
    // r      = perturbed yaw velocity [rad/s]
    // psi    = perturbed yaw angle [rad]
    // xpos   = position in x-direction [m]
    // ypos   = position in y-direction [m]
    // delta  = actual rudder angle [rad]
    
    // ui = delta_c, commanded rudder angle [rad]
    // Reference: M.S. Chislett and J. Stroem-Tejsen (1965)
    //            Planar Motion Mechanism Tests and Full-Scale Steering
    //            and Maneuvering Predictions for a Mariner Class Vessel,
    //            Technical Report Hy-5
    //            Hydro- and Aerodynamics Laboratory, Lyngby, Denmark.
    if (x.length != 7) {
        return []
    }

    // Nominal Velocity [m/s]
    U0 = 7.7175; // m/s, about 15 knots
    V0 = 0.0;
    
    // Normalization variables
    L = 160.93;
    U = Math.sqrt(Math.pow(U0 + x[0], 2) + Math.pow(x[1],2));
    
    // Non-dimensional states and inputs
    delta_c = ui;
    
    u     = x[0]/U;
    v     = x[1]/U;
    r     = x[2]*L/U;
    psi   = x[3];
    delta = x[6];
    
    // Parameters, hydrodynamic derivatives and main dimensions
    delta_max = 10.0 // max rudder angle [deg]
    Ddelta_max = 5.0 // max rudder derivative [deg/s]
    
    m  = 798e-5;
    Iz = 39.2e-5;
    xG = -0.023;
    
    Xudot =  -42e-5;   Yvdot =  -748e-5;   Nvdot = 4.646e-5;
    Xu    = -184e-5;   Yrdot =-9.354e-5;   Nrdot = -4.38e-5;
    Xuu   = -110e-5;   Yv    = -1160e-5;   Nv    =  -264e-5;
    Xuuu  = -215e-5;   Yr    =  -499e-5;   Nr    =  -166e-5;
    Xvv   = -899e-5;   Yvvv  = -8078e-5;   Nvvv  =  1636e-5;
    Xrr   =   18e-5;   Yvvr  = 15356e-5;   Nvvr  = -5483e-5;
    Xdd   =  -95e-5;   Yvu   = -1160e-5;   Nvu   =  -264e-5;
    Xudd  = -190e-5;   Yru   =  -499e-5;   Nru   =  -166e-5;
    Xrv   =  798e-5;   Yd    =   278e-5;   Nd    =  -139e-5;
    Xvd   =   93e-5;   Yddd  =   -90e-5;   Nddd  =    45e-5;
    Xuvd  =   93e-5;   Yud   =   556e-5;   Nud   =  -278e-5;                   
                       
    Yuud  =   278e-5;   Nuud  =  -139e-5;                   
    Yvdd  =    -4e-5;   Nvdd  =    13e-5;                   
    Yvvd  =  1190e-5;   Nvvd  =  -489e-5;                   
    Y0    =    -4e-5;   N0    =     3e-5;                   
    Y0u   =    -8e-5;   N0u   =     6e-5;                   
    Y0uu  =    -4e-5;   N0uu  =     3e-5;
    
    
    // Masses and moments of inertia
    m11 = m-Xudot;
    m22 = m-Yvdot;
    m23 = m*xG-Yrdot;
    m32 = m*xG-Nvdot;
    m33 = Iz-Nrdot;
    
    // Rudder saturation and dynamics
    if (Math.abs(delta_c) > delta_max*Math.PI/180) {
        if (delta_c > 0) {
            delta_c = delta_max*Math.PI/180
        } else {
            delta_c = -delta_max*Math.PI/180
        }
    }
    
    delta_dot = delta_c - delta

    if (Math.abs(delta_dot) > Ddelta_max*Math.PI/180) {
        if (delta_dot > 0) {
            delta_dot = Ddelta_max*Math.PI/180
        } else {
            delta_dot = -Ddelta_max*Math.PI/180
        }
    }
    // Forces and moments
    X = Xu*u + Xuu*Math.pow(u,2) + Xuuu*Math.pow(u,3) + Xvv*Math.pow(v,2) + Xrr*Math.pow(r,2) + Xrv*r*v + Xdd*Math.pow(delta,2) +
        Xudd*u*Math.pow(delta,2) + Xvd*v*delta + Xuvd*u*v*delta
    Y = Yv*v + Yr*r + Yvvv*Math.pow(v,3) + Yvvr*Math.pow(v,2)*r + Yvu*v*u + Yru*r*u + Yd*delta + 
        Yddd*Math.pow(delta,3) + Yud*u*delta + Yuud*Math.pow(u,2)*delta + Yvdd*v*Math.pow(delta,2) + 
        Yvvd*Math.pow(v,2)*delta + (Y0 + Y0u*u + Y0uu*Math.pow(u,2))
    N = Nv*v + Nr*r + Nvvv*Math.pow(v,3) + Nvvr*Math.pow(v,2)*r + Nvu*v*u + Nru*r*u + Nd*delta + 
        Nddd*Math.pow(delta,3) + Nud*u*delta + Nuud*Math.pow(u,2)*delta + Nvdd*v*Math.pow(delta,2) + 
        Nvvd*Math.pow(v,2)*delta + (N0 + N0u*u + N0uu*Math.pow(u,2))
    
    // Dimensional state derivative
    xdot = [X*(Math.pow(U,2)/L)/m11,
            -(-m33*Y+m23*N)*(Math.pow(U,2)/L)/(m22*m33-m23*m32),
            (-m32*Y+m22*N)*(Math.pow(U,2)/Math.pow(L,2))/(m22*m33-m23*m32),
            r*(U/L),
            (Math.cos(psi)*(U0/U+u) - Math.sin(psi)*v)*U,
            (Math.sin(psi)*(U0/U+u) + Math.cos(psi)*v)*U,
            delta_dot];
    
    return [xdot, delta_c]
}

function advanceOneStep() {
    [dState,ui] = dX(state,ui)
    state = state.map(function(num, idx) {
        return num + dState[idx]*0.2
    });
}

function draw() {
    // canvas 800x800
    // real diameter 10000x10000
    context = canvas.getContext("2d");
    context.clearRect(0, 0, 1000, 1000);

    // context.fillStyle = "rgb(200, 100, 220)";
    // context.fillRect(50, 50, 100, 100); // obstacle

    // plot xpos, ypos
    context.save();
    context.translate(state[4]/12.5, state[5]/12.5);
    context.rotate(state[3]);
    context.drawImage(ship, -50, -50 * ship.height / ship.width, 100, 100 * ship.height / ship.width);    
    context.restore();
}

function keyup_handler(event)
{
    if(event.keyCode == 87 || event.keyCode == 83)
    {
        // state[6] = 0;
    }
}

function keypress_handler(event)
{
    console.log(event.keyCode);
    // if(event.keyCode == 87)
    // {
    //     mod = 1;
    // }
    // if(event.keyCode == 83)
    // {
    //     mod = -1;
    // }
    if(event.keyCode == 65)
    {
        ui += 0.08;
    }
    if(event.keyCode == 68)
    {
        ui -= 0.08;
    }
}