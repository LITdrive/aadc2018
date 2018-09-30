#pragma once

#include <boost/property_tree/ptree.hpp>

#include <string>
#include <map>

using namespace std;

class FilePropertiesObserver
{
public:
	FilePropertiesObserver(string path);

	/**
     * \brief Read the file and store all the updated properties
     * \param subsample Only read the file every nth time this method will be called (default: 1, every time)
     */
	void ReloadProperties();

	/**
	 * \brief Get the a float value from the [float] section of the ini file.
	 * If the value doesn't exist, 0 will be returned.
	 * Don't forget to call ReloadProperties() to refresh the values.
	 * \param propertyName The key of the value
	 * \return The float value of the property
	 */
	float GetFloat(string propertyName);

	/**
	* \brief Get the a int value from the [int] section of the ini file.
	* If the value doesn't exist, 0 will be returned.
	 * Don't forget to call ReloadProperties() to refresh the values.
	* \param propertyName The key of the value
	* \return The int value of the property
	*/
	int GetInt(string propertyName);

	/**
	 * \brief Call this function at a point where you eventually want to
	 * update the properties (e.g. the Process method). However, you can
	 * add a subsampleFactor, that will only reload the init file when
	 * you called this trigger subsampleFactor times.
	 * This reduces the i/o load within methods, that are called very frequently.
	 * \param subsampleFactor Only read the file every nth time this method will be called
	 * (default: 1, every time)
	 */
	void TriggerPropertiesReload(int subsampleFactor);

private:

	string m_path;
	int m_reload_call_counter;
	boost::property_tree::ptree* m_property_tree;
	map<string, float> m_float_store;
	map<string, int> m_int_store;
};
