function flight_state_dot_array = calc_state_dot(object, t, flight_state_array)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
def calculate_state_dot(self, t, flight_state_array):
        self.rocket_state = RocketState.init_from_array(array=flight_state_array, t=t,
                                                        rocket=self.flight_model.rocket)
        flight_step = FlightStep(t=t, flight=self.flight_model, rocket_state=self.rocket_state)

        env = self.env  # load environment

        g = self.env.g  # gravitational constant
        mass = self.rocket_state.mass
        mass = mass + (self.flight_model.mass_rocket_engineless_factor - 1.) * (
            self.flight_model.rocket.rocket_engineless.mass)  # change mass due to factor for grid searches with
        # different masses
        state = flight_state_array  # state carries the spatial
        launch_rail_length = self.flight_model.launch_environment.launch_location.launch_rail_length

        # define state vectors for better readability
        # all coordinates are in WORLD coordinate system with z-axis pointing UP
        X = state[0:3]  # position of rocket
        Q = Rot.from_quat(state[3:7]).as_quat()  # quaternions
        V = state[7:10]  # velocity of center of mass
        W = state[10:13]  # angular velocity

        # rotation matrix from earth to rocket coordinates
        rotmat = Rot.from_quat([Q])

        # Rocket axes wrt earth coord
        #YA = rotmat.apply([0, 0, 1]).reshape(3)
        #PA = rotmat.apply([0, 1, 0]).reshape(3)
        RA = rotmat.apply([1, 0, 0]).reshape(3) # roll axis is x-axis in rocket frame

        # velocity ---------------------------------------------------------
        Xdot = V

        # Q angular velocity ------------------------------------------------
        I_mat = np.array(  # momen
            [[self.rocket_state.I_xx, 0, 0], [0, self.rocket_state.I_yy, 0], [0, 0, self.rocket_state.I_zz]])

        # transform from angular velocity to quaternions. be careful with changes!
        s = Q[3]   # scalar part of quaternion
        v = Q[0:3]  # vector part of quaternion
        sdot = -0.5 * (np.dot(W, v))   # attention with signs!
        vdot = 0.5 * (s * W + np.cross(W, v))
        Qdot = np.concatenate((vdot, sdot), axis=None)

        # Angle of Attack ----------------------------------------------------
        X_cog = self.rocket_state.CoG_x  # center of gravity

        # Wind: ignore wind before launch rail exit
        if X[2] < launch_rail_length:
            Wind = np.array([0, 0, 0])
        else:
            Wind = env.W(X[2])

        # calculate current mach number
        self.rocket_state.speed_of_sound = env.C_from_altitude_AGL(X[2])
        mach_number = np.linalg.norm(V) / self.rocket_state.speed_of_sound
        self.rocket_state.mach_number = mach_number

        t_burnout = np.array(self.flight_model.rocket.engine.t_values)[-1]  # TODO: define t_burnout in a smarter way

        run_controller = False
        control_step_prior = None
        if self.flight_model.controller is not None:
            if len(self.control_steps) > 0:
                control_step_prior = self.control_steps[-1]
                if t - control_step_prior.t >= 1 / self.flight_model.controller.sampling_rate:
                    run_controller = True
            else:
                run_controller = True

        run_state_estimation = False
        state_estimation_step_prior = None
        if self.state_estimator_model is not None:
            if len(self.state_estimation_steps) > 0:
                state_estimation_step_prior = self.state_estimation_steps[-1]
                if t - state_estimation_step_prior.t >= 1 / self.flight_model.state_estimator.sampling_rate:
                    run_state_estimation = True
            else:
                run_state_estimation = True

        # control input----------------------------------
        flight_step.control_step = None
        if run_controller:
            if self.prior_flight_step is not None:
                flight_step_for_control_step = self.prior_flight_step
            else:
                flight_step_for_control_step = flight_step
            tracking_feedback = flight_step_for_control_step.rocket_state.airbrake_state[0]
            if self.state_estimator_model is None or state_estimation_step_prior is None:
                flight_step.control_step = self.controller.run(t, control_step_prior,
                                                               rocket_state=flight_step_for_control_step.rocket_state,
                                                               state_estimation_step=None,
                                                               tracking_feedback=tracking_feedback)
            else:
                flight_step.control_step = self.controller.run(t, control_step_prior,
                                                               rocket_state=None,
                                                               state_estimation_step=self.state_estimation_steps[-1],
                                                               tracking_feedback=tracking_feedback)
            if flight_step.control_step is not None:
                self.control_steps.append(flight_step.control_step)

        # simulate airbrake
        airbrake_state_dot = self.airbrake_simulator.run(flight_step.rocket_state, control_step_prior,
                                                         self.simulate_airbrake)

        # --------------------------------------------------------------

        # calculate CoP for a first time to get alpha. CoP will later be calculated again using alpha as input
        X_cop = self.C_values.CoP_x_interp(self.rocket_state.mach_number, 0)
        X_stab = X_cop - X_cog

        V_cop = V  # before launch rail exit V_cop is the same as V_cm
        # if the rocket rotates, V_cop differs from V_cm by V_omega (rotation of CoP around axis)
        if (np.linalg.norm(W) > 0.000001) and (X[2] > launch_rail_length):
            V_omega = X_stab * np.cross(RA, W)
            V_cop = V + V_omega

        # the apparent air velocity is a combination of V_cop and Wind.
        # this velocity is used for angle of attack calculation and for aerodynamic force calculations
        V_app = V_cop + Wind
        V_app_abs = np.linalg.norm(V_app)  # absolute value of air velocity

        # calculate velocity unit vector
        if X[2] < launch_rail_length:
            V_norm = RA  # before lr exit V is aligned with rocket
        else:
            if V_app_abs != 0.:
                V_norm = V_app / V_app_abs
            else:
                V_norm = V_app
        if (V_app_abs == 0) or (X[2] < launch_rail_length):  # before lr exit AOA is 0
            alpha = 0
        else:
            if np.abs(np.dot(V_norm, RA)) > 1:  # prevent runtime error due to dot product larger than 1
                alpha = 0.
            else:
                alpha = np.arccos(np.dot(V_norm, RA))

        # P forces---------------------------------------------
        Fthrust = self.flight_model.rocket.engine.thrust_interp(
            # import current motor thrust and rotate along rocket axis
            t) * RA * self.flight_model.engine_efficiency  # multiplication by engine_efficiency as correction factor

        mg = mass * g
        Fg = np.array([0, 0, -mg])  # gravity force

        # ---- calculate current aerodynamic properties at current flight state
        rocket_diameter = self.flight_model.rocket.rocket_engineless.diameter
        A_ref = (rocket_diameter / 2) ** 2 * np.pi  # reference area of rocket(body)

        if t < t_burnout:  # C_A is axial drag coefficient. differentiate between C_A before and after burnout
            C_A = self.C_values.C_A_power_on_interp(self.rocket_state.mach_number,
                                                    alpha) * self.flight_model.drag_coefficient_factor
        else:
            C_A = self.C_values.C_A_interp(self.rocket_state.mach_number,
                                           alpha) * self.flight_model.drag_coefficient_factor

        C_A_AB = self.airbrake_simulator.get_drag_coefficient(C_A_rocket=C_A, velocity=V_app_abs,
                                                              mach_number=V_app_abs/self.rocket_state.speed_of_sound,
                                                              airbrake_extension=self.rocket_state.airbrake_state[0])
        C_A = C_A + C_A_AB

        C_N_alpha = self.C_values.C_N_alpha_interp(self.rocket_state.mach_number, alpha)
        X_cop = self.C_values.CoP_x_interp(self.rocket_state.mach_number, alpha)  # center of pressure location
        X_stab = X_cop - X_cog  # stability length
        SSM = X_stab / rocket_diameter  # static stability margin

        # ------

        # check if SSM is above 0
        if (SSM < 0) and (t < t_burnout):
            raise Exception("SSM is smaller than 0: unstable rocket, calculation stopped")

        # axial drag force
        Famag = 0.5 * env.rho(X[2]) * A_ref * V_app_abs ** 2 * C_A  # * np.cos(alpha)
        Fa = - Famag * RA #* np.sign(V)

        # normal drag force
        Fnmag = 0.5 * env.rho(X[2]) * A_ref * V_app_abs ** 2 * C_N_alpha * alpha
        RA_Vplane = np.cross(RA, V_norm)
        if np.linalg.norm(RA_Vplane) != 0.:  # normalize RA_Vplane
            RA_Vplane = RA_Vplane / np.linalg.norm(RA_Vplane)
        Fn = Fnmag * np.cross(RA, RA_Vplane)

        if (Fthrust[2] < mg) and (X[2] < launch_rail_length):  # no takeoff yet
            Ftot = np.array([0, 0, 0])
        elif (Fthrust[2] >= mg) and (
                X[2] < launch_rail_length):  # takeoff but not launch rail exit yet: no aerodynamic forces
            Ftot = Fg + Fthrust
        else:
            Ftot = Fg + Fthrust + Fa + Fn

        #  Torque -------------------------------------------
        C_1 = 0.5 * env.rho(X[2]) * A_ref * V_app_abs ** 2 * C_N_alpha * X_stab  # C_1 is correction coefficient
        Trq_n = C_1 * alpha * RA_Vplane  # normal torque

        length = self.flight_model.rocket.rocket_engineless.length
        if X[2] > launch_rail_length:  # no torque before launch rail exit
            Trq = np.array([0, 0, 0])
            # TODO myself, @Maurice
            C_2A = 0.5 * env.rho(X[2]) * V_app_abs * A_ref * (
                    2 * (X_cog - 0.425) ** 2 + (C_N_alpha - 2) * (
                    X_cog - ((X_cop * C_N_alpha - 0.425 * 2) / (C_N_alpha - 2))) ** 2)

            # C_2A is aerodyn damping coefficient
            C_2R = -1 * self.flight_model.rocket.engine.mass_derivative_interp(t) * (
                    length - X_cog) ** 2  # C_2R is thrust damping coefficient
            C_2 = (C_2A + C_2R)  # total damping from thrust and aerodynamics

            natural_frequency = np.sqrt(C_1 / I_mat[1][1])

            # print(I_mat[0][0],I_mat[1][1],I_mat[2][2])
            damping_ratio = C_2 / (2 * np.sqrt(C_1 * I_mat[1][1]))

            Trq_damp = - C_2 * rotmat.apply(np.array([0, 1, 1]) * rotmat.inv().apply(W).reshape(3)).reshape(3)
            
            # Trq_launch_lug = 0.5 * env.rho(X[2]) * 0.01*0.02 * V_app_abs ** 2 * 1 * rocket_diameter/2. * PA #Launch
            # lug torque with 2x1cm ll with Cd = 1
            
            # add roll torque ------------
            # modeled equilibrium roll speed W_roll_ideal is due to fin cant angel (or similar).
            # Is assumed to be 1deg/s per 1m/s velocity for EULER
            W_roll_ideal = np.linalg.norm(V) / 360 * 2*np.pi
            W_roll_current = np.dot(W, RA)  # determine roll component of current W -> projection of W onto RA

            # the difference in angular roll velocity, as vector oriented along RA
            W_roll_diff = (W_roll_ideal - W_roll_current) * RA

            # normalization of roll-torque such that equilibrium roll velocity is reached in 0.2s
            Trq_roll = 5 * W_roll_diff * I_mat[0][0]
            # ------------

            Trq = Trq_n + Trq_damp #+ Trq_roll  # + Trq_launch_lug
            
            invIbody = np.linalg.inv(I_mat)
            W_dot = rotmat.apply(np.dot(invIbody, rotmat.inv().apply(Trq).reshape(3))).reshape(3)

        else:
            Trq = np.array([0, 0, 0])
            damping_ratio = natural_frequency = 0
            W_dot = np.array([0, 0, 0])

        flight_state_dot_array = np.concatenate(
            [Xdot, Qdot, Ftot / mass, W_dot, airbrake_state_dot])  # collect all derivatives for DGL solver

        # write calculated values to rocket_state to save later
        self.rocket_state.Famag = Famag

        self.rocket_state.update_from_flight_state_dot_array(
            flight_state_dot_array)  # write derivative to rocket state to save later
        self.rocket_state.CoP_x = X_cop
        self.rocket_state.SSM = SSM
        self.rocket_state.C_d = C_A
        self.rocket_state.C_N_alpha = C_N_alpha
        self.rocket_state.natural_frequency = natural_frequency
        self.rocket_state.damping_ratio = damping_ratio
        self.rocket_state.angle_of_attack = alpha  # * np.sign(V_norm[0] - RA[0])

        # state estimation
        flight_step.state_estimation_step = None
        if run_state_estimation:
            control_step_for_state_estimation = None
            if len(self.control_steps) > 0:
                control_step_for_state_estimation = self.control_steps[-1]
            flight_step_simulated_sensor_data = self.sensor_data_simulator.step(flight_step=flight_step)
            flight_step.state_estimation_step = self.state_estimator.step(t,
                                                                          flight_step_simulated_sensor_data,
                                                                          state_estimation_step_prior,
                                                                          control_step_for_state_estimation)
            self.simulated_sensor_data.extend(flight_step_simulated_sensor_data)
            self.state_estimation_steps.append(flight_step.state_estimation_step)

        if len(self.flight_steps) > 0:
            if (t - self.flight_steps[-1].t > self.dt_max) or flight_step.control_step is not None \
                    or flight_step.state_estimation_step is not None:
                self.rocket_states.append(self.rocket_state)
                self.flight_steps.append(flight_step)
        else:
            self.rocket_states.append(self.rocket_state)
            self.flight_steps.append(flight_step)

        self.prior_flight_step = flight_step

        if self.flight_analysis_queue is not None:
            self.push_to_flight_analysis_queue()

        print("\r  -->  step #", len(self.flight_steps),
              ". Flight time: {:.3f}. ".format(t) +
              "Altitude: {:.1f} m ".format(X[2]) + "              ", end="")

        return flight_state_dot_array
end

