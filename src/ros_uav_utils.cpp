
/**
 * @brief Randomize the UAV parameters
 * Reads a list of parameters for randomization from the parameter server
 * 
 */
void randomizeUavParameters(ros::NodeHandle n)
{
	// Read which parameters should be randomized
	XmlRpc::XmlRpcValue parameterList;
	int i;
	char paramMsg[50];
	double std_dev;
	sprintf(paramMsg, "paramRandomizer/std_dev");
	if (!n.getParam(paramMsg, std_dev))
	{
		ROS_INFO("paramRandomizer/std_dev parameter not found. Setting to 0");
		std_dev = 0;
	}
	else
	{
		ROS_INFO("Requested UAV parameter randomization by %f", std_dev);
	}
	if (std_dev)
	{
		ROS_INFO("Randomizing parameter std_dev by %f", std_dev);
		sprintf(paramMsg, "paramRandomizer/param_names");
		if(!n.getParam(paramMsg, parameterList))
		{
			ROS_WARN("Could not find a list of parameters to randomize in the param server!");
		}
		else
		{
			for (i = 0; i < parameterList.size(); ++i)
			{
				ROS_ASSERT(parameterList[i].getType() == XmlRpc::XmlRpcValue::TypeString);
				randomizeParameter(n, static_cast<std::string>(parameterList[i]), std_dev);
			}
		}
	}
	else
	{
		ROS_INFO("Aircraft parameter randomization not requested");
	}
}

/**
 * @brief Randomize a double parameter by a given standard deviation
 * 
 * @param paramName The parameter name
 * @param std_dev The variance to alter the parameter by
 */
void randomizeParameter(ros::NodeHandle n, std::string paramName, double std_dev)
{
	double param, randomizedParam;
	if (!n.getParam(paramName, param))
	{
		ROS_ERROR("Unable to retrieve parameter %s", paramName.c_str());
	}
	else
	{
		std::default_random_engine generator;
		std::normal_distribution<double> distribution(0.0, std_dev);

		randomizedParam = param*(1 + distribution(generator));
		ROS_INFO("Randomizing parameter %s", paramName.c_str());
		n.setParam(paramName, randomizedParam);
	}

}