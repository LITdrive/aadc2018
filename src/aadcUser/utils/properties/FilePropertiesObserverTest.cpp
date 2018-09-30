#include "FilePropertiesObserver.h"

#include <iostream>

int main()
{
	FilePropertiesObserver fpo("test_file.ini");

	fpo.ReloadProperties();
	cout << fpo.GetFloat("p") << endl;
	cout << fpo.GetFloat("i") << endl;
	cout << fpo.GetFloat("d") << endl;

	cout << fpo.GetInt("x") << endl;
	cout << fpo.GetInt("y") << endl;
	cout << fpo.GetInt("z") << endl;

	return 0;
}
