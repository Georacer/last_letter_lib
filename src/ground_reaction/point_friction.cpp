
//////////////////////////////////
// Define PointFriction class
//////////////////////////////////

// Class constructor
PointFriction::PointFriction(YAML::Node config, YAML::Node worldConfig) : GroundReaction(config, worldConfig)
{
	vector<double> doubleVect;
	double temp[6];
	// Read contact points number from parameter server
	getParameter(config, "contactPtsNo", contactPtsNo);
	// Create an appropriately sized matrix to contain contact point information
	pointCoords.setZero(3, contactPtsNo); // contact points coordinates in the body frame
	materialIndex.setZero(contactPtsNo, 1); // contact points material type index
	springIndex.setZero(2,contactPtsNo); // contact points spring characteristics
	len=0.2;

	// Set coefficient of friction for each material
	frict[0] = 0.7;
	frict[1] = 0.4;
	frict[2] = 1.0;
	frict[3] = 0.4; //To update composite to ground firction coefficients!

	// Read contact points location and material from parameter server
	for (int j = 0; j<contactPtsNo; j++) { //Distribute the data
		string contactPointName = "contactPoint" + std::to_string(j+1);
		doubleVect.clear();
		getParameterList(config, contactPointName, doubleVect);

		pointCoords(0, j) = doubleVect[0]; // Save body frame contact point coordinates
		pointCoords(1, j) = doubleVect[1];
		pointCoords(2, j) = doubleVect[2];
		materialIndex(j) = doubleVect[3]; // A separate contact point material index array
		springIndex(0, j) = doubleVect[4];
		springIndex(1, j) = doubleVect[5]; // And the spring constants

	}

	// Create and initialize spring contraction container
	spp.setZero(contactPtsNo, 1);
	// Create and initialize previous spring contraction container
	sppprev.setZero(contactPtsNo, 1);

	contact = false;

	// Create other matrices needed for calculations
	cpi_up.setZero(3, contactPtsNo); // upper spring end matrix
	cpi_down.setZero(3, contactPtsNo); // lower spring end matrix
	spd.setZero(contactPtsNo, 1); // spring contraction speed
}

// Class destructor
PointFriction::~PointFriction()
{
}

// Wrench_t calculation function
Vector3d PointFriction::getForce(const SimState_t states, const WrenchSum_t wrenchSum)
{
	double Reb[9];
	double mu;
	int i, j;
	contact = false;
	safe = true; // NaN protection flag
	Vector3d Euler;

	Quaterniond orientation = states.pose.orientation; // Read vehicle orientation quaternion
	Euler = quat2euler(orientation);

	Eigen::MatrixXd normalF;
	normalF.setZero(contactPtsNo, 1);
	Eigen::MatrixXi contactList;
	contactList.setZero(contactPtsNo, 1);

	Wrench_t tempE, totalE; 
	Vector3d we,vpoint,Ve;
	Eigen::MatrixXd vpointList;
	vpointList.setZero(3, contactPtsNo);

	// Get velocity in the inertial frame
	Ve = orientation.inverse() * states.velocity.linear;
	// Read vehicle coordinates
	uavpos = states.pose.position;

	// Rotate contact points coordinates from body frame to earth frame
	// and place the spring ends to the contact points positions
	// Place the upper spring end to contact point
	cpi_up = Eigen::Translation<double, 3>(uavpos) * orientation.inverse() * pointCoords;
	// Raise the upper spring end to obtain a visually pleasing result in RViz
	cpi_up.row(2).array() -= len;
	cpi_down = cpi_up;

	// Rotate body angular speeds to earth frame
	we = orientation.inverse() * states.velocity.angular;

	// Main calculations for each contact point
	cpi_down.row(2).array() += len; // Place lower spring end "len" below upper spring end
	// Calculate force arm
	Eigen::MatrixXd dx = cpi_up-uavpos.replicate(1, contactPtsNo);

	for (i=0;i<contactPtsNo;i++)
	{
		if (cpi_down(2,i)>0) // if the lower spring end touches the ground
		{
			contact=true; // update contact flag
			contactList(i)=1; // update list of point which contact the ground

			cpi_down(2,i)=0; // Force lower spring end to stay at ground level
			spp(i)=len+cpi_up(2,i); // Calculate current spring contraction
			spd(i)=(spp(i)-sppprev(i))/dt; // Calculate current spring contraction spreed
			Vector3d dxElement = dx.col(i);
			vpoint = Ve + we.cross(dxElement); // Contact point Earth-frame speed due vehicle velocity and rotation
			vpointList.block<3,1>(0,i) = vpoint; // Store the contact point velocity

			// Calculate spring force. Acts only upon Earth z-axis
			double forceZ = -(springIndex(0,i)*spp[i] + springIndex(1,i)*spd[i]);

			// Make spring force negative or zero, as the spring lower end isn't fixed on the ground
			if (forceZ > 0)
				forceZ = 0;
			/** @todo model full spring contraction (possibly with exponential force curve after full contraction?) */
			normalF(i) = forceZ; // store the normal force on each contact point

		}

		// If there is no ground contact
		else {
			// Set spring contraction to zero
			spp(i) = 0;
			spd(i) = 0;
		}
		sppprev(i) = spp(i);
	}

	// Measure the sum of the normal force and count the touching points
	double totalN = 0;
	int totalPoints = 0;
	totalN = normalF.sum();
	totalPoints = contactList.sum();

	// Distribute the friction forces
	for (i=0;i<contactPtsNo;i++)
	{
		if (contactList(i)) // If this point is contacting the ground
		{

			int tempIndex = materialIndex(i);
			// Get longitudinal and lateral friction coefficients for this surface
			mu = frict[tempIndex];
			double Fmax = mu*normalF(i);

			// Calculate force arm
			dx.col(i) = cpi_up.block<3,1>(0,i) - uavpos;

			double forceX, forceY, forceZ;

			// Calculate point friction in the Earth frame
			if (vpointList.block<3,1>(0,i).norm() > 0.01) // If the point is moving
			{
				double forceAngle = atan2(vpointList(i, 1), vpointList(i, 0));
				forceX = Fmax*cos(forceAngle);
				forceY = Fmax*sin(forceAngle);
			}
			else // If static friction is applied on the point
			{
				// Read the external forces and torques
				extForce = wrenchSum.wrenchGrav.force
							+ wrenchSum.wrenchProp.force
							+ wrenchSum.wrenchAero.force;
				// Rotate forces to the inertial frame
				extForce = orientation.conjugate() * extForce;

				extTorque = wrenchSum.wrenchGrav.torque
							+ wrenchSum.wrenchProp.torque
							+ wrenchSum.wrenchAero.torque;
				// Rotate torque to the inertial frame
				extTorque = orientation.conjugate() * extTorque;

				// Calculate the force arm in the Earth coordinates
				Vector3d dx_inertial = orientation.conjugate() * dx.col(i);

				Vector3d reaction2Force, reaction2Torque;
				if (std::fabs(totalN) > 1e-6)
				{
					reaction2Force(0) = -extForce.x()*(normalF(i)/totalN); // Calculate the load share of the point
					reaction2Force(1) = -extForce.y()*(normalF(i)/totalN);
				}
				else
				{
					// No action required
				}

				Vector3d dx_inertial_planar;
				dx_inertial_planar = dx_inertial;
				dx_inertial_planar(2) = 0;
				double armLength = dx_inertial_planar.norm(); // Calculate arm length...
				dx_inertial_planar.normalize(); // ... and the unit arm vector

				Vector3d extTorque_planar(0, 0, -extTorque.z()/totalPoints);
				double torqueNorm = extTorque_planar.norm(); // Calculate torque norm...
				extTorque_planar.normalize(); // ... and the unit torque vector

				reaction2Torque = extTorque_planar.cross(dx_inertial_planar); // Calculate unit force vector
				reaction2Torque *= (torqueNorm/armLength); // Calculate the torque share of the point

				Vector3d reaction, unit_reaction;
				reaction = reaction2Force + reaction2Torque; // Sum the force components
				double reactionNorm = reaction.norm(); // Calculate the friction force magnitude
				reactionNorm = std::min(Fmax,reactionNorm); // and compare with the static friction limit
				unit_reaction = reaction.normalized();
				reaction = reactionNorm * unit_reaction;

				tempE.force(0) = reaction.x();
				tempE.force(1) = reaction.y();
			}

			tempE.force[2] = normalF(i);

			// Add current contact point force contribution to the total
			totalE.force += tempE.force;
			// Calculate current contact point torque
			Vector3d dxElement = dx.col(i);
			tempE.torque = dxElement.cross(tempE.force);
			// Add current contact point torque contribution to the total
			totalE.torque += tempE.torque;
		}
	}

	// Chech for rogue NaN results
    if (!totalE.force.allFinite())
	{
		safe = false; /** @todo remove the safe safety, it is not used*/
		throw runtime_error("State NAN in PointFriction force calculation!");
	}

    if (!totalE.torque.allFinite())
    {
		safe = false;
		throw runtime_error("State NAN in PointFriction torque calculation!");
	}

	// If there is a ground contact write the resulting wrench
	if (contact and safe) {
		wrenchGround.force= orientation * totalE.force;
		wrenchGround.torque= orientation * totalE.torque;
	}
	// Otherwise there is no wrench created
	else {
		wrenchGround.force.setZero();
		wrenchGround.torque.setZero();
	}

	return wrenchGround.force;

}

// Dummy torque calculation function
Vector3d PointFriction::getTorque(const SimState_t states, const WrenchSum_t wrenchSum)
{

	return wrenchGround.torque;
}