#include "FilePropertiesObserver.h"

#include <boost/property_tree/ini_parser.hpp>
#include <iostream>

FilePropertiesObserver::FilePropertiesObserver(string path) : m_path(path), m_reload_call_counter(0),
                                                              m_property_tree(new boost::property_tree::ptree())
{
	ifstream f(path.c_str());
	if (!f.good())
	{
		cerr << "The file " + path + " doesn't exist." << endl;
		m_path = "";
	}
}

void FilePropertiesObserver::ReloadProperties()
{
	if (!m_path.empty())
	{
		read_ini(m_path, *m_property_tree);
	}
}


void FilePropertiesObserver::TriggerPropertiesReload(int subsampleFactor)
{
	m_reload_call_counter++;
	if (m_reload_call_counter % subsampleFactor == 0)
	{
		m_reload_call_counter = 0;
		ReloadProperties();
	}
}

float FilePropertiesObserver::GetFloat(string propertyName)
{
	return m_property_tree->get("float." + propertyName, std::numeric_limits<float>::quiet_NaN());
}

int FilePropertiesObserver::GetInt(string propertyName)
{
	return m_property_tree->get("int." + propertyName, 0);
}
