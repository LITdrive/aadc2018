/*********************************************************************
Copyright (c)
Audi Autonomous Driving Cup. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: “This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.”
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2017-05-11 16:43:59#$ $Rev:: 63082   $
**********************************************************************/

#pragma once

#include <string.h>
#include <vector>

namespace aadc
{
    namespace jury
	{

		/*! the enum for the different states of the car which have to be sent to the jury*/
		enum stateCar{
			statecar_error = -1,
			statecar_ready = 0,
			statecar_running = 1,
			statecar_complete = 2,
			statecar_startup = -2
		};

        inline std::string stateCarToString(stateCar state)
        {
            switch (state)
            {
            case statecar_error:
                return std::string("error");
                break;
            case statecar_ready:
                return std::string("ready");
                break;
            case statecar_running:
                return std::string("running");
                break;
            case statecar_complete:
                return std::string("complete");
                break;
            case statecar_startup:
                return std::string("startup");
                break;
            default:
                return std::string("invalid");
                break;
            }
        }

		/*! the enum for the different action which are sent from the jury to the car */
		enum juryAction{
			action_stop=-1,
			action_getready=0,
			action_start=1
		};
		
		/*! the enum for maneuvers which can be contained in the maneuver list*/
		enum maneuver{
			manuever_undefined = 0,
			maneuver_left = 1,
			maneuver_right = 2,
			maneuver_straight = 4,
			maneuver_parallel_parking = 5,
			maneuver_cross_parking = 6,
			maneuver_pull_out_left = 7,
			maneuver_pull_out_right = 8,
			maneuver_merge_left = 9,
			maneuver_merge_right = 10
		};
		
        inline std::string maneuverToString(maneuver man)
		{
			switch (man)
			{
				case maneuver_left:
					return std::string("left");
					break;
                case maneuver_right:
                    return std::string("right");
                    break;
                case maneuver_straight:
                    return std::string("straight");
                    break;
                case maneuver_parallel_parking:
                    return std::string("parallel_parking");
                    break;
                case maneuver_cross_parking:
                    return std::string("cross_parking");
                    break;
                case maneuver_pull_out_left:
                    return std::string("pull_out_left");
                    break;
                case maneuver_pull_out_right:
                    return std::string("pull_out_right");
                    break;
                case maneuver_merge_left:
                    return std::string("merge_left");
                    break;
                case maneuver_merge_right:
                    return std::string("merge_right");
                    break;
				default:
                    return std::string("");
					break;					
			}
		}
		
        inline maneuver maneuverFromString(std::string man)
		{
			if (man.compare("left") == 0)
			{
                return maneuver_left;
			}
            else if (man.compare("right") == 0)
            {
                return maneuver_right;
            }
            else if (man.compare("straight") == 0)
            {
                return maneuver_straight;
            }
            else if (man.compare("parallel_parking") == 0)
            {
                return maneuver_parallel_parking;
            }
            else if (man.compare("cross_parking") == 0)
            {
                return maneuver_cross_parking;
            }
            else if (man.compare("pull_out_left") == 0)
            {
                return maneuver_pull_out_left;
            }
            else if (man.compare("pull_out_right") == 0)
            {
                return maneuver_pull_out_right;
            }
            else if (man.compare("merge_left") == 0)
            {
                return maneuver_merge_left;
            }
            else if (man.compare("merge_right") == 0)
            {
                return maneuver_merge_right;
            }
		    return manuever_undefined;
		}		
		
		/*! one maneuver with its id, action enum and an extra field (e.g. used for parking lot ids) */
		struct tManeuver
		{
			int id;
			maneuver action;
            int extra;

            tManeuver()
            {
                id = 0;
                action = manuever_undefined;
                extra = 0;
            }

		};

		/*! one sector which contains different maneuvers */
		struct tSector
		{
			int id;
			std::vector<tManeuver> sector;
		};
		
		/*! the maneuver list which contains multiple sectors */
		typedef std::vector<tSector> maneuverList;
		
		enum juryContainerId: int 
		{
			container_id_unkown = 0,
			container_id_juryStruct,
			container_id_maneuverlist,
			container_id_opendrive_map,
			container_id_traffic_sign_map
		};
		
		struct tJuryContainer
		{
			juryContainerId id;
			int dataSize;
			char* data;

        };

        static bool serializeContainer(const tJuryContainer& container, std::vector<char>& serialized)
        {
            serialized.clear();
            if (container.dataSize <= 0)
            {
                return false;
            }
            serialized.resize(sizeof(container.id) + sizeof(container.dataSize) + container.dataSize);
            size_t dataOffset = serialized.size() - container.dataSize;
            //copy container header
            memcpy(serialized.data(), &container, dataOffset);
            //copy payload
            memcpy(serialized.data() + dataOffset, container.data, container.dataSize);
            return true;
        }

        static bool deserializeContainer(const std::vector<char>& serialized, tJuryContainer& container)
        {
            if (serialized.size() < sizeof tJuryContainer::id + sizeof tJuryContainer::dataSize + 1)
            {   //make sure the we have at least one byte of data
                return false;
            }
            //first int is the id
            container.id = *reinterpret_cast<const aadc::jury::juryContainerId*>(&serialized[0]);
            //second is the size
            container.dataSize = *reinterpret_cast<const int*>(&serialized[4]);
            if (static_cast<size_t>(container.dataSize) != serialized.size() - (sizeof(container.id) + sizeof(container.dataSize)))
            {   //inconsitency in payload and container size
                return false;
            }
            //then dynamic sized data
            container.data = reinterpret_cast<char*>(malloc(container.dataSize));
            memcpy(container.data, &serialized[8], container.dataSize);
            return true;
        }
	
	}
}
