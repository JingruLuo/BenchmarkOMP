/*
 * BenchmarkSettings.cpp
 *
 *  Created on: Sep 2, 2014
 *      Author: jingru
 */
#include "BenchmarkSettings.h"
#include <fstream>
using namespace std;

void writeExperimentInfo(char *experimentDir, ScenarioIndex scenarioIndex) {
	char costFile[100];
	sprintf(costFile, "%s/best-path-cost.txt", experimentDir);
	ofstream file;
	file.open(costFile);
	file << "nD\twidth\tthin\tbest-path-length\n";
	for (int i = 0; i < num_nD_to_test; i++) {
		int nD = nD_to_test[i];
		for (int j = 0; j < num_width_to_test; j++) {
			double width = width_to_test[j];
			for (int k = 0; k < num_thin_to_test; k++) {
				double thin = thin_to_test[k];

				double bestpathcost = getBestPathCost(scenarioIndex, nD, width,	thin);
				file << nD << "\t" << width << "\t" << thin << "\t"	<< bestpathcost << endl;
			}
		}
	}
	file.close();

	char summaryfilename[100];
	sprintf(summaryfilename, "%s/summary.txt", experimentDir);
	file.open(summaryfilename);
	file << "cutoff_time_2D:" << cutoff_seconds_2D << endl;
	file << "cutoff_time_3D:" << cutoff_seconds_3D << endl;
	file << "cutoff_time_HD:" << cutoff_seconds_HD << endl;
	file << "nRuns:" << nRuns << endl;

	file << "num_nD_to_test:" << num_nD_to_test << endl;
	for (int i = 0; i < num_nD_to_test; i++) {
		file << nD_to_test[i] << ";";
	}
	file << endl;
	file << "num_width_to_test:" << num_width_to_test << endl;
	for (int i = 0; i < num_width_to_test; i++) {
		file << width_to_test[i] << ";";
	}
	file << endl;
	file << "num_thin_to_test:" << num_thin_to_test << endl;
	for (int i = 0; i < num_thin_to_test; i++) {
		file << thin_to_test[i] << ";";
	}
	file << endl;

	file << "planners that are running:"<<endl;
	file << "runMRRT:"<<runMRRT<<endl;
	file << "runMRRT_shortcut:"<<runMRRT_shortcut<<endl;
	file << "runRRTStar:"<<runRRTStar<<endl;
	file << "runPRMStar:"<<runPRMStar<<endl;
	file << "runPRMStar_Lazy:"<<runPRMStar_Lazy<<endl;
	file << "runCE:"<<runCE<<endl;
	file << "runFMM:"<<runFMM<<endl;
	file << "runFMMOffset:"<<runFMMOffset<<endl;

	file << "FMMOffsetRangeMax:"<<FMMOffsetRangeMax<<endl;
	file << "FMMOffsetRangeMin:"<<FMMOffsetRangeMin<<endl;

	if (scenarioIndex == NarrowPassage_2_homotopy)
		file << "scenarioindex:NarrowPassage_2_homotopy" << endl;
	else if (scenarioIndex == NarrowPassage_1_homotopy)
		file << "scenarioindex:NarrowPassage_1_homotopy" << endl;
	else if(scenarioIndex == PlanaryLinkage)
		file << "scenarioindex:PlanaryLinkage"<<endl;
	else if(scenarioIndex == NarrowKink_1_homotopy)
		file << "scenarioindex:NarrowKink_1_homotopy"<<endl;

	file << "FMMOffsetRange:" << FMMOffsetRangeMax << endl;
	if(scenarioIndex == PlanaryLinkage){
		file << "joint_start_5D:"<<endl;
		for(int i = 0; i < 5; i++)
			file << joint_start_5D[i]<<";";
		file<<endl;

		file << "joint_start_6D:"<<endl;
		for(int i = 0; i < 6; i++)
			file << joint_start_6D[i]<<";";
		file<<endl;

		file << "joint_start_7D:"<<endl;
		for(int i = 0; i < 7; i++)
			file << joint_start_7D[i]<<";";
		file<<endl;

		file << "joint_start_8D:"<<endl;
		for(int i = 0; i < 8; i++)
			file << joint_start_8D[i]<<";";
		file<<endl;
	}

	file.close();

}

bool createTopDirs(char *experimentDir) {
	if(createDir(experimentDir) == false)
		return false;

	char topDir[100];
	if(runMRRT){
		sprintf(topDir, "%s/%s", experimentDir, MRRTFolder.c_str());
		if(createDir(topDir) == false)
			return false;
	}
	if(runMRRT_shortcut){
		sprintf(topDir, "%s/%s", experimentDir, MRRTFolder_shortcut.c_str());
		if(createDir(topDir) == false)
			return false;
	}
	if(runRRTStar){
		sprintf(topDir, "%s/%s", experimentDir, RRTStarFolder.c_str());
		if(createDir(topDir) == false)
			return false;
	}
	if(runCE){
		sprintf(topDir, "%s/%s", experimentDir, CEFolder.c_str());
		if(createDir(topDir) == false)
			return false;
	}
	if(runPRMStar){
		sprintf(topDir, "%s/%s", experimentDir, PRMStarFolder.c_str());
		if(createDir(topDir) == false)
			return false;
	}
	if(runPRMStar_Lazy){
		sprintf(topDir, "%s/%s", experimentDir, PRMStarFolder_Lazy.c_str());
		if(createDir(topDir) == false)
			return false;
	}
	if(runFMM){
		sprintf(topDir, "%s/%s", experimentDir, FMMFolder.c_str());
		if(createDir(topDir) == false)
			return false;
	}
	if(runFMMOffset){
		sprintf(topDir, "%s/%s", experimentDir, FMMOffsetFolder.c_str());
		if(createDir(topDir) == false)
			return false;
	}
	return true;
}



