#pragma once

typedef std::pair<float, float> FLOAT_LOC;

/// <summary>
/// Class that encapsulates interface to Depot data in ECTSP problem
/// It contains members:
///		- UUID
///		- Location (X,Y pair)
/// </summary>

class Depot {

public:
	Depot(int inUuid, float inX, float inY)
		: mUuid(inUuid)
	{
		mLocation = std::make_pair(inX, inY);
	}

	const std::string getPrintName() const
	{
		return std::to_string(mUuid);
	}

	const std::string getPrintableData() const
	{
		std::string _result;
		_result = "  ";
		_result += getPrintName();
		_result += "\t";
		_result += std::to_string(getLocation().first);
		_result += "\t";
		_result += std::to_string(getLocation().second);
		return _result;
	}
	/** Returns location of the depot
	* \return Location of the depot as float,float pair
	*/
	const std::pair<float, float> getLocation() const
	{
		return mLocation;
	}

private:
	// Unique identifier of Depot
	int mUuid;

	// Location
	FLOAT_LOC mLocation;
};
