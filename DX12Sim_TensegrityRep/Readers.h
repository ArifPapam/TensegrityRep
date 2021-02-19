///////////////////////////////////
// Arif Pramudwiatmoko           //
// Konagaya Laboratory           //
// Tokyo Institute fo Technology //
///////////////////////////////////

/*This file is for reading PDB files.*/

#pragma once

#include <string>
#include <fstream>
#include <iostream>
#include <vector>

using namespace std;

namespace PDBreader
{
	struct Atom {
		bool isHetAtm;
		int serial;
		string name;
		char altLoc;
		string resName;
		char chainID;
		int resSeq;
		char iCode;
		float x, y, z, occupancy, tempFactor;
		string element, charge;
	};

	class Atoms {
	public:
		vector<Atom> atom;
		void addAtom(string, bool);
		int getNumAtoms();
	};

	struct Connect {
		int atomSN;
		int bondSN[4];
	};

	class Connects {
	public:
		vector<Connect> connect;
		void addConnect(string);
		int getNumConnects();
	};

	class FileReader {
	private:
		string filename, content;
		ifstream file;
	public:
		FileReader(string);
		~FileReader();
		void init();
		string getFilename();
		string readAll();
		//string readHeader(string);
		Atoms atoms;
		Connects connects;
	};
}