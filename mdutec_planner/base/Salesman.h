#pragma once

#include <vector>
#include <string>

class Salesperson
{
public:

	Salesperson(int inUuid, float inX, float inY, float inSpeed, std::vector<bool> inColors)
		: mUuid(inUuid),
		mSpeed(inSpeed),
		mColors(inColors)
	{
		mStartLocation = std::make_pair(inX, inY);
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
		_result += std::to_string(getStartLocation().first);
		_result += "\t";
		_result += std::to_string(getStartLocation().second);
		_result += "\t";
		_result += getPrintableColorCodes();
		_result += "\t";
		_result += std::to_string(mSpeed);
		return _result;
	}
	const std::vector<bool> getColors() const
	{
		return mColors;
	}

	const std::string getPrintableColorCodes() const
	{
		std::string _result;
		for (int _i = 0; _i < mColors.size(); _i++)
		{
			if (mColors.at(_i))
			{
				_result += (_result.empty() ? std::to_string(_i + 1) : " " + std::to_string(_i + 1));
			}
		}
		return _result;
	}
	/** Returns start location of the salesperson
	* \return Starting location of the salesperson as float,float pair
	*/
	const std::pair<float, float> getStartLocation() const
	{
		return mStartLocation;
	}

	/** Checks color presence by checking 
	* \param inColor
	* \return True if color is in allowed colors of salesperson
	*/
	const bool hasColor(int inColor) const
	{
		
		return (inColor - 1) <= mColors.size() && inColor > 0 
			&& mColors.at((size_t)inColor - 1);

		/*return (inColor < mColors.size() && inColor > 0
			&& mColors.at((size_t)inColor - 1);*/
	}

	/** Returns speed of the salesperson
	* \return Speed of the salesperson
	*/
	const float getSpeed() const
	{
		return mSpeed;
	}

private:
	// Unique identifier of Salesperson
	int mUuid;

	// Location
	std::pair<float, float> mStartLocation;

	// Supported colors
	std::vector<bool> mColors;

	// Speed of traveling
	float mSpeed;
};
