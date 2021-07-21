//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef _hectormaprepmultimap_h__
#define _hectormaprepmultimap_h__

#include "MapRepresentationInterface.h"
#include "MapProcContainer.h"

#include "../map/GridMap.h"
#include "../map/OccGridMapUtilConfig.h"
#include "../matcher/ScanMatcher.h"

#include "../util/DrawInterface.h"
#include "../util/HectorDebugInfoInterface.h"


namespace hectorslam{

class MapRepMultiMap : public MapRepresentationInterface
{

public:
  MapRepMultiMap(float mapResolution, int mapSizeX, int mapSizeY, unsigned int numDepth, const Eigen::Vector2f& startCoords, DrawInterface* drawInterfaceIn, HectorDebugInfoInterface* debugInterfaceIn)
  {
    //unsigned int numDepth = 3;
    Eigen::Vector2i resolution(mapSizeX, mapSizeY);

    float totalMapSizeX = mapResolution * static_cast<float>(mapSizeX);
    float mid_offset_x = totalMapSizeX * startCoords.x();

    float totalMapSizeY = mapResolution * static_cast<float>(mapSizeY);
    float mid_offset_y = totalMapSizeY * startCoords.y();

    std::cout<< "numDepth "<<numDepth<<std::endl;
    for (unsigned int i = 0; i < numDepth; ++i){
      //std::cout << "HectorSM map lvl " << i << ": cellLength: " << mapResolution << " res x:" << resolution.x() << " res y: " << resolution.y() << "\n";
      GridMap* gridMap = new hectorslam::GridMap(mapResolution,resolution, Eigen::Vector2f(mid_offset_x, mid_offset_y));
      //std::cout << "new hectorslam::GridMap OK " <<std::endl;
      OccGridMapUtilConfig<GridMap>* gridMapUtil = new OccGridMapUtilConfig<GridMap>(gridMap);
      //std::cout << "new OccGridMapUtilConfig OK " <<std::endl;
      ScanMatcher<OccGridMapUtilConfig<GridMap> >* scanMatcher = new hectorslam::ScanMatcher<OccGridMapUtilConfig<GridMap> >(drawInterfaceIn, debugInterfaceIn);
      std::cout << "new hectorslam::ScanMatcher OK " <<std::endl;

      mapContainer.push_back(MapProcContainer(gridMap, gridMapUtil, scanMatcher));

      std::cout << "mapContainer.push_back OK " <<std::endl;

      resolution /= 2;
      mapResolution*=2.0f;
    }

    //dataContainers.resize(numDepth-1);

	dataContainers_1.resize(numDepth-1);
	dataContainers_2.resize(numDepth-1);

  }

  virtual ~MapRepMultiMap()
  {
    unsigned int size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i){
      mapContainer[i].cleanup();
    }
  }

  virtual void reset()
  {
    unsigned int size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i){
      mapContainer[i].reset();
    }
  }

  virtual float getScaleToMap() const { return mapContainer[0].getScaleToMap(); };

  virtual int getMapLevels() const { return mapContainer.size(); };
  virtual const GridMap& getGridMap(int mapLevel) const { return mapContainer[mapLevel].getGridMap(); };

  virtual void addMapMutex(int i, MapLockerInterface* mapMutex)
  {
    mapContainer[i].addMapMutex(mapMutex);
  }

  MapLockerInterface* getMapMutex(int i)
  {
    return mapContainer[i].getMapMutex();
  }

  virtual void onMapUpdated()
  {
    unsigned int size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i){
      mapContainer[i].resetCachedData();
    }
  }

  //=====Anhung==================================================================//
  //virtual Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld, const DataContainer& dataContainer, Eigen::Matrix3f& covMatrix)
  virtual Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld, const DataContainer& dataContainer_1, const DataContainer& dataContainer_2, int& device_num, Eigen::Matrix3f& covMatrix)
  {
	  //std::cout<<"matchData  ==="<<std::endl;
	  size_t size = mapContainer.size();

	  Eigen::Vector3f tmp(beginEstimateWorld);

	  for (int index = size - 1; index >= 0; --index){
		  //std::cout << " m " << i;
		  if (index == 0){
			  tmp  = (mapContainer[index].matchData(tmp, dataContainer_1, dataContainer_2, device_num, covMatrix, 5));
		  }else{
			  dataContainers_1[index-1].setFrom(dataContainer_1, static_cast<float>(1.0 / pow(2.0, static_cast<double>(index))));

			  if(device_num == 2)
				dataContainers_2[index-1].setFrom(dataContainer_2, static_cast<float>(1.0 / pow(2.0, static_cast<double>(index))));


			  tmp  = (mapContainer[index].matchData(tmp, dataContainers_1[index-1], dataContainers_2[index-1], device_num, covMatrix, 3));
		  }
	  }
	  return tmp;



	/*
    size_t size = mapContainer.size();

    Eigen::Vector3f tmp(beginEstimateWorld);

    for (int index = size - 1; index >= 0; --index){
      //std::cout << " m " << i;
      if (index == 0){
        tmp  = (mapContainer[index].matchData(tmp, dataContainer, covMatrix, 5));
      }else{
        dataContainers[index-1].setFrom(dataContainer, static_cast<float>(1.0 / pow(2.0, static_cast<double>(index))));
        tmp  = (mapContainer[index].matchData(tmp, dataContainers[index-1], covMatrix, 3));
      }
    }
    return tmp;
	*/
  }
  //=====Anhung==================================================================//



  virtual void updateByScan(const DataContainer& dataContainer_1, const DataContainer& dataContainer_2, int& device_num, const Eigen::Vector3f& robotPoseWorld)
  {
	  unsigned int size = mapContainer.size();

	  for (unsigned int i = 0; i < size; ++i){
		  if (i==0){
			  mapContainer[i].updateByScan(dataContainer_1, dataContainer_2, device_num, robotPoseWorld);
		  }else{
			  mapContainer[i].updateByScan(dataContainers_1[i-1], dataContainers_2[i-1], device_num, robotPoseWorld);
		  }
	  }
  }


  virtual void setUpdateFactorFree(float free_factor)
  {
    size_t size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i){
      GridMap& map = mapContainer[i].getGridMap();
      map.setUpdateFreeFactor(free_factor);
    }
  }

  virtual void setUpdateFactorOccupied(float occupied_factor)
  {
    size_t size = mapContainer.size();

    for (unsigned int i = 0; i < size; ++i){
      GridMap& map = mapContainer[i].getGridMap();
      map.setUpdateOccupiedFactor(occupied_factor);
    }
  }

  /*=====Anhung========================Set Map Value =============================*/
  virtual void setMapValue(int &container_index, int &value_index, float &value)
  {
	  mapContainer[container_index].gridMap->setMapValue(value_index, value);
  }

  /*=====Anhung========================Get Map prob =============================*/
  virtual float getMapProb(int &container_index, int &value_index){

	  return mapContainer[container_index].gridMap->getMapProb(value_index);

  }


  /*=====Anhung=================Calculate error of mapping ======================*/
  virtual float CalculateMappingError(const DataContainer& dataContainer_1, const DataContainer& dataContainer_2, int& device_num, const Eigen::Vector3f& robotPoseWorld)
  {

    //mapPose       車子位置    世界  pixel為單位
    //scanEndMapf   雷射點      世界  pixel為單位


	  //std::cout<<"======MapRepMultiMap   CalculateMappingError========= "<<std::endl;
	  float occupied_thresh_value = 0.65;
	  int container_index = 0;

	  float return_error = 0.0;
	  float count = 0.0;


	  //Get pose in map coordinates from pose in world coordinates
	  Eigen::Vector3f mapPose(mapContainer[0].gridMap->getMapCoordsPose(robotPoseWorld));

	  //Get a 2D homogenous transform that can be left-multiplied to a robot coordinates vector to get world coordinates of that vector
	  Eigen::Affine2f poseTransform((Eigen::Translation2f(
										 mapPose[0], mapPose[1]) * Eigen::Rotation2Df(mapPose[2])));

    //std::cout<<"======mapPose========= "<<mapPose<<std::endl;

	  //Get number of valid beams in current scan
	  int numValidElems_1 = dataContainer_1.getSize();

	  int numValidElems_2 = dataContainer_2.getSize();

	  //Iterate over all valid laser beams
	  for (int i = 0; i < numValidElems_1; ++i) {

		  //Get map coordinates of current beam endpoint
		  Eigen::Vector2f scanEndMapf(poseTransform * (dataContainer_1.getVecEntry(i)));
		  //std::cout << "scanEndMapf" << scanEndMapf << "\n";

		  //add 0.5 to beam endpoint vector for following integer cast (to round, not truncate)
		  scanEndMapf.array() += (0.5f);

		  //Get integer map coordinates of current beam endpoint
		  Eigen::Vector2i scanEndMapi(scanEndMapf.cast<int>());

		  int Offset = scanEndMapi.y() * mapContainer[0].gridMap->getSizeX() + scanEndMapi.x();

		  float prob = getMapProb(container_index, Offset);

      //std::cout<<"====== mapContainer[0].gridMap->getSizeX()========= "<< mapContainer[0].gridMap->getSizeX()<<std::endl;
      //td::cout<<"======container_index========= "<<container_index<<std::endl;
        //std::cout<<"======Offset========= "<<Offset<<std::endl;


		  if(prob > occupied_thresh_value){
			return_error += 1.0;
		  }

		  count += 1;
	  }


	  if(device_num == 2){

		  for (int i = 0; i < numValidElems_2; ++i) {

			  //Get map coordinates of current beam endpoint
			  Eigen::Vector2f scanEndMapf(poseTransform * (dataContainer_2.getVecEntry(i)));
			  //std::cout << "\ns\n" << scanEndMapf << "\n";

			  //add 0.5 to beam endpoint vector for following integer cast (to round, not truncate)
			  scanEndMapf.array() += (0.5f);

			  //Get integer map coordinates of current beam endpoint
			  Eigen::Vector2i scanEndMapi(scanEndMapf.cast<int>());

			  int Offset = scanEndMapi.y() * mapContainer[0].gridMap->getSizeX() + scanEndMapi.x();

			  float prob = getMapProb(container_index, Offset);

			  if(prob > occupied_thresh_value){
				  return_error += 1.0;
			  }

			  count += 1;
		  }

	  }

	  if(count == 0)
		  return 0.0;
	  else{
		 return_error = return_error/count;
		 return return_error;
	  }

  }



protected:
  std::vector<MapProcContainer> mapContainer;

  //=====Anhung==================================================================//
  std::vector<DataContainer> dataContainers_1;
  std::vector<DataContainer> dataContainers_2;
};

}

#endif
