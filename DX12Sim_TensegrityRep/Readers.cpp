///////////////////////////////////
// Arif Pramudwiatmoko           //
// Konagaya Laboratory           //
// Tokyo Institute fo Technology //
///////////////////////////////////

/*This file is for reading PDB files.*/

#include "Readers.h"
#include <sstream>

/////////////////////////Atoms class definition/////////////////////////
void PDBreader::Atoms::addAtom(string s, bool het) {
	Atom a;
	a.isHetAtm = het;
	a.serial = stoi(s.substr(6, 5));
	a.name = s.substr(12, 4);
	a.altLoc = s.at(16);
	a.resName = s.substr(17, 3);
	a.chainID = s.at(21);
	a.resSeq = stoi(s.substr(22, 4));
	a.iCode = s.at(26);
	a.x = stof(s.substr(30, 8));
	a.y = stof(s.substr(38, 8));
	a.z = stof(s.substr(46, 8));
	a.occupancy = stof(s.substr(54, 6));
	a.tempFactor = stof(s.substr(60, 6));
	a.element = s.substr(76, 2);
	a.charge = s.substr(78, 2);

	atom.push_back(a);
}

int PDBreader::Atoms::getNumAtoms() {
	return atom.size();
}

/////////////////////////Connects class definition/////////////////////////
void PDBreader::Connects::addConnect(string s) {
	Connect c;
	c.atomSN = stoi(s.substr(6, 5));
	c.bondSN[0] = stoi(s.substr(11, 5));
	try {
		c.bondSN[1] = stoi(s.substr(16, 5));
	}
	catch (exception& e) {
		c.bondSN[1] = 0;
		c.bondSN[2] = 0;
		c.bondSN[3] = 0;
	}
	try {
		c.bondSN[2] = stoi(s.substr(21, 5));
	}
	catch (exception& e) {
		c.bondSN[2] = 0;
		c.bondSN[3] = 0;
	}
	try {
		c.bondSN[3] = stoi(s.substr(26, 5));
	}
	catch (exception& e) {
		c.bondSN[3] = 0;
	}
	connect.push_back(c);
}

int PDBreader::Connects::getNumConnects() {
	return connect.size();
}

/////////////////////////FileReader class definition/////////////////////////
PDBreader::FileReader::FileReader(string fname) {
	filename = fname;
	file.open(filename);
	if (file.is_open())
		cout << "File " << filename << " is opened" << endl;
	else
		cout << "File opening fail!!!" << endl;
	init();
}

PDBreader::FileReader::~FileReader() {
	file.close();
}

void PDBreader::FileReader::init() {
	string temp;
	while (getline(file, temp)) {
		content += temp + "\n";
		if (!temp.compare(0, 6, "ATOM  "))
			atoms.addAtom(temp, false);
		else if (!temp.compare(0, 6, "HETATM"))
			atoms.addAtom(temp, true);
		else if (!temp.compare(0, 6, "CONECT"))
			connects.addConnect(temp);
	}
}
string PDBreader::FileReader::getFilename() {
	return filename;
}

string PDBreader::FileReader::readAll() {
	return content;
}

/*string FileReader::readHeader(string header) {
	string data, temp;
	//file.seekg(0, ios::beg);
	cout << "file.tellg() = " << file.tellg() << endl;
	while (getline(file, temp)) {
		if (!temp.compare(0, 6, header))
			data += temp + "\n";
	}
	return data;
}*/