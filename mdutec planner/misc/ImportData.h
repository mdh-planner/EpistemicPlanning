#pragma once
#include <vector>
#include "../solution_representations/TSolution.h"

class ImportData
{
public:

	static MissionData loadData(string& _dataExtension) {

		int _numberOfColors = 4;

		string _path = "../../../data/";

		MissionData _data;

		loadSalespersonData(_data.salesUnit, _dataExtension, _path, _numberOfColors);

		loadCityData(_data.cities, _dataExtension, _path, _numberOfColors);

		loadDepotData(_data.depots, _dataExtension, _path);

		return _data;
	}

	static void printLoadedData(MissionData& _data) {

		cout << endl << endl;

		std::cout << "Salesperson" << "\t" << "X" << "\t\t" << "Y" << "\t\t" << "Colors" << "\t" << "Velocity" << std::endl;
		for(Salesperson _salesman : _data.salesUnit)
		{
			std::cout << _salesman.getPrintableData() << std::endl;
		}

		cout << endl << endl;

		std::cout << "City\tX\t\tY\t\tColor\tDuration\tPrecedence" << std::endl;
		for (City _city : _data.cities)
		{
			std::cout << _city.getPrintableData() << std::endl;
		}

		cout << endl << endl;

		std::cout << "Depot\tX\t\tY" << std::endl;
		for (Depot _depot : _data.depots)
		{
			std::cout << _depot.getPrintableData() << std::endl;
		}
	}

private:

	static void loadSalespersonData(std::vector<Salesperson>& _salesUnit, string& _datasetExtension, string& _path, int _numberOfColors) {

		vector<vector<string>> _salespersonRawData;
		string _salespersonDataFilePath(_path + "Salespersons" + _datasetExtension);

		// Load salesperson data
		cout << "Loading salesperson data from:\t" << _salespersonDataFilePath << endl;
		ifstream _in(_salespersonDataFilePath.c_str());
		if (!_in.is_open()) {
			cout << "Salesperson path cannot be open." << endl;
			return;
		}

		_salespersonRawData = readCSV(_in);
		cout << "Done reading file." << endl;
		cout << "Loading to memory." << endl;

		/// Assumption is data structure being column-wise: UUID, X, Y, Color set, Speed
		for (long i = 1; i < _salespersonRawData.size(); i++) {
			auto& _dataEntry = _salespersonRawData[i];
			int _uuid = stoi(_dataEntry[0]);
			float _x = stof(_dataEntry[1]);
			float _y = stof(_dataEntry[2]);

			auto _readColors = [](string str, int _numberOfColors) {

				std::vector<bool> _colors(_numberOfColors, false);
				istringstream iss(str);

				for (auto _it = istream_iterator<string>(iss); _it != istream_iterator<string>(); ++_it)
				{
					int _val = stoi(*_it);
					// Incrementing by 1 because color data starts with 0 :(
					_val += 1;
					if (_val > _numberOfColors || _val < 1)
					{
						cout << "Problem with color input. Value found " << _val << " is outside of range [1 - " << _numberOfColors << "]" << endl;
					}
					else
						_colors.at((size_t)_val - 1) = true;
				}
				return _colors;
			};

			auto _colors = _readColors(_dataEntry[3], _numberOfColors);
			float _speed = stof(_dataEntry[4]);

			Salesperson _m(_uuid, _x, _y, _speed, _colors);
			_salesUnit.push_back(_m);
		}
		cout << "Done loading salespersons" << endl;
	}

	static void loadCityData(std::vector<City>& _cities, string& _datasetExtension, string& _path, int _numberOfColors) {

		vector<vector<string>> _cityRawData;
		string _cityDataFilePath(_path + "Cities" + _datasetExtension);

		// Load city data

		cout << "Loading city data from:\t" << _cityDataFilePath << endl;
		ifstream in(_cityDataFilePath.c_str());
		if (!in.is_open()) {
			cout << "City path cannot be open." << endl;
			return;
		}

		_cityRawData = readCSV(in);
		cout << "Done reading file." << endl;
		cout << "Loading to memory." << endl;

		/// Assumption is data structure being column-wise: UUID, X, Y, Duration, Color, Precendece
		for (long i = 1; i < _cityRawData.size(); i++) {
			auto& _dataEntry = _cityRawData[i];
			int _uuid = stoi(_dataEntry[0]);
			float _x = stof(_dataEntry[1]);
			float _y = stof(_dataEntry[2]);

			float _duration = stof(_dataEntry[3]);
			int _color = stoi(_dataEntry[4]);
			int _precedenceCity = stoi(_dataEntry[5]);

			// Incrementing by 1 because color data starts with 0 :(
			_color += 1;
			if (_color > _numberOfColors || _color < 1) {
				cout << "Problem with color input. Value found " << _color << " is outside of range [1 - " << _numberOfColors << "]" << endl;
			}

			City _city(_uuid, _x, _y, _color, _duration, _precedenceCity);
			_cities.push_back(_city);
		}
		cout << "Done loading cities" << endl;
	}

	static void loadDepotData(std::vector<Depot>& _depots, string& _datasetExtension, string& _path) {

		vector<vector<string>> _depotsRawData;
		string _depotDataFilePath(_path + "Depots" + _datasetExtension);

		// Load depot data

		cout << "Loading depots data from:\t" << _depotDataFilePath << endl;
		ifstream in(_depotDataFilePath.c_str());
		if (!in.is_open()) {
			cout << "Depot path cannot be open." << endl;
			return;
		}

		_depotsRawData = readCSV(in);
		cout << "Done reading file." << endl;
		cout << "Loading to memory." << endl;

		/// Assumption is data structure being column-wise: UUID, X, Y
		for (long i = 1; i < _depotsRawData.size(); i++) {

			auto& _dataEntry = _depotsRawData[i];
			int _uuid = stoi(_dataEntry[0]);
			float _x = stof(_dataEntry[1]);
			float _y = stof(_dataEntry[2]);

			Depot _depot(_uuid, _x, _y);
			_depots.push_back(_depot);
		}
		cout << "Done loading depots" << endl;
	}
};