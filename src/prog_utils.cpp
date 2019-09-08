#include <iostream>
// #include <string>
// #include <stdexcept>

#include "prog_utils.hpp"


bool startsWith(std::string mainStr, std::string startStr)
{
    if (mainStr.find(startStr) == 0)
        return true;
    else
        return false;
}

// Filter a YAML::Node file to keep only a sub-parameter set
// Example: for start string /world, keep /world/timeControls but not /environment/rho
YAML::Node filterConfig(YAML::Node config, std::string prefix)
{
    YAML::Node filteredNode;
    for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
        string paramName = it->first.as<string>();
        if (startsWith(paramName, prefix)) {
            string newName = paramName.erase(0, prefix.size());
            filteredNode[newName] = it->second;
        }
        else
        {
        }
    }
    return filteredNode;
}

// Randomize the requested parameters of a configuration node by std_dev
YAML::Node randomizeParameters(YAML::Node config, vector<string> stringVec, double std_dev)
{
    if (std_dev < 0)
    {
        throw domain_error("Standard deviation cannot be negative");
    }

    std::random_device rdev;
    std::default_random_engine generator;
    generator.seed(rdev());
    std::normal_distribution<double> distribution(0.0, std_dev);
    YAML::Node newConfig(config);

    if (std_dev) // Apply only if std_dev>0
    {
        double scalarParam;
        for (auto const& paramString: stringVec)
        {
            if (!config[paramString].Type() == YAML::NodeType::Scalar)
            {
                throw runtime_error("Requested parameter "+paramString+" is not scalar.");
            }
            cout << "Randomizing parameter: " << paramString << endl;
            getParameter(config, paramString, scalarParam);
            newConfig[paramString] = scalarParam * (1+distribution(generator));
        }
    }
    return newConfig;
}


/////////////////////////////////////////////////////////////////
// Build a new polynomial, reading from a configuration YAML Node
Polynomial * buildPolynomial(YAML::Node config)
{
	int polyType;
    getParameter(config, "polyType", polyType);
	std::cout<< "building a new polynomial: ";
	switch (polyType)
	{
	case 0: {
		std::cout << "selecting 1D polynomial" << std::endl;
        int polyNo;
        getParameter(config, "polyNo", polyNo);
        std::vector<double> coeffVect;
        getParameterList(config, "coeffs", coeffVect); 
        if (coeffVect.size() != (polyNo+1))
        {
            throw runtime_error("Parameter array coeffs size did not match polyNo");
        }
		double coeffs[polyNo+1];
        std::copy(coeffVect.begin(), coeffVect.end(), coeffs);
		return new Polynomial1D(polyNo, coeffs);
		}
	case 1: {
		std::cout << "selecting 2D polynomial" << std::endl;
        std::vector<int> polyNoVect;
        getParameterList(config, "polyNo", polyNoVect);
        std::vector<double> coeffVect;
		int polyOrder1 = coeffVect[0];
		int polyOrder2 = coeffVect[1];
        getParameterList(config, "coeffs", coeffVect);
        if (coeffVect.size() != ((2*polyOrder2 + 2*polyOrder1*polyOrder2 + polyOrder1 - polyOrder1*polyOrder1 + 2)/2))
        {
            throw runtime_error("Parameter array coeffs size did not match polyNo");
        }
		double coeffs[coeffVect.size()];
        std::copy(coeffVect.begin(), coeffVect.end(), coeffs);
		return new Polynomial2D(polyOrder1, polyOrder2, coeffs);
		}
	case 2: {
		std::cout << "selecting cubic spline" << std::endl;
        int breaksNo;
        getParameter(config, "breaksNo", breaksNo);
        std::vector<double> breaksVect;
        getParameterList(config, "breaks", breaksVect);
        if (breaksVect.size() != (breaksNo+1))
        {
            throw runtime_error("Spline breaks order and provided breaks number do not match");
        }
		double breaks[breaksVect.size()];
        std::copy(breaksVect.begin(), breaksVect.end(), breaks);
        std::vector<double> coeffVect;
        getParameterList(config, "coeffs", coeffVect);
		if (breaksNo*4 != coeffVect.size()) {
			throw runtime_error("breaks order and provided coeffs number do not match");
		}
		double coeffs[coeffVect.size()];
        std::copy(coeffVect.begin(), coeffVect.end(), coeffs);
		return new Spline3(breaksNo, breaks, coeffs);
		}
	default: {
		throw runtime_error("Error while constructing a polynomial");
		break;
		}
	}
}

