/*
 * ParamValWrongType.cpp
 *
 *  Created on: Dec 19, 2011
 *      Author: hammonds
 */

#include "ParamValWrongType.h"

ParamValWrongType::ParamValWrongType(const std::string& description):
    std::logic_error(description){

}

