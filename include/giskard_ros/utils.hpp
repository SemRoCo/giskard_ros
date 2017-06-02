/*
* Copyright (C) 2016-2017 Georg Bartels <georg.bartels@cs.uni-bremen.de>
*
* This file is part of giskard.
*
* giskard is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* as published by the Free Software Foundation; either version 2
* of the License, or (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/


#ifndef __GISKARD_UTILS_HPP__
#define __GISKARD_UTILS_HPP__

#include <functional>
#include <string>

namespace giskard { namespace ros
{
  template<class T>
  inline size_t calculateHash(const T& msg)
  {
    std::stringstream ss;
    ss << msg;
    std::hash<std::string> hash_fn;
    return hash_fn(ss.str());
  }

}}

#endif // __GISKARD_UTILS__HPP
