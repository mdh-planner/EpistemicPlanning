#pragma once

typedef std::pair<float, float> FLOAT_LOC;

/// <summary>
/// Class that encapsulates interface to City data in ECTSP problem
/// It contains members:
///		- UUID
///		- Location (X,Y pair)
///		- Duration
///		- Color
///		- Precedence condition
/// </summary>
class City {

public:
	City(int inUuid, float inX, float inY, int inColor, float inDuration, int inPrecedence)
		: mUuid(inUuid),
		mColor(inColor),
		mDuration(inDuration),
		mPrecedenceCondition(inPrecedence)
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
		_result += "\t";
		_result += std::to_string(getColor());
		_result += "\t";
		_result += std::to_string(mDuration);
		_result += "\t";
		_result += std::to_string(mPrecedenceCondition);
		return _result;
	}
	/** Returns location of the city
	* \return Location of the city as float,float pair
	*/
	const std::pair<float, float> getLocation() const
	{
		return mLocation;
	}

	/** Returns color of the city
	* \return Color index of the city
	*/
	const int getColor() const
	{
		return mColor;
	}

	/** Returns duration of stay in the city for any salesperson
	* \return Duration of stay in the city
	*/
	const float getDurationOfStay() const
	{
		return mDuration;
	}

	/** Return precedence condition UUID
	* \return Uuid of the preceding city
	*/
	const int getPrecedenceCondition() const
	{
		return mPrecedenceCondition;
	}

private:
	// Unique identifier of City
	int mUuid;

	// Location
	FLOAT_LOC mLocation;
	
	// City's //color
	int mColor;

	// Duration
	float mDuration;

	// Precedence Condition
	int mPrecedenceCondition;
};
