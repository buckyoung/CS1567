/* Software License Agreement (BSD License)
 *
 * Copyright (c) 2011, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 *  * Neither the name of Willow Garage, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Auto-generated by gensrv_cpp from file /home/student/cs1567_ws/src/cs1567p1/srv/MakeNewMaze.srv
 *
 */


#ifndef CS1567P1_MESSAGE_MAKENEWMAZE_H
#define CS1567P1_MESSAGE_MAKENEWMAZE_H

#include <ros/service_traits.h>


#include <cs1567p1/MakeNewMazeRequest.h>
#include <cs1567p1/MakeNewMazeResponse.h>


namespace cs1567p1
{

struct MakeNewMaze
{

typedef MakeNewMazeRequest Request;
typedef MakeNewMazeResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct MakeNewMaze
} // namespace cs1567p1


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::cs1567p1::MakeNewMaze > {
  static const char* value()
  {
    return "15d28abd61347f29f8562c138de059b4";
  }

  static const char* value(const ::cs1567p1::MakeNewMaze&) { return value(); }
};

template<>
struct DataType< ::cs1567p1::MakeNewMaze > {
  static const char* value()
  {
    return "cs1567p1/MakeNewMaze";
  }

  static const char* value(const ::cs1567p1::MakeNewMaze&) { return value(); }
};


// service_traits::MD5Sum< ::cs1567p1::MakeNewMazeRequest> should match 
// service_traits::MD5Sum< ::cs1567p1::MakeNewMaze > 
template<>
struct MD5Sum< ::cs1567p1::MakeNewMazeRequest>
{
  static const char* value()
  {
    return MD5Sum< ::cs1567p1::MakeNewMaze >::value();
  }
  static const char* value(const ::cs1567p1::MakeNewMazeRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::cs1567p1::MakeNewMazeRequest> should match 
// service_traits::DataType< ::cs1567p1::MakeNewMaze > 
template<>
struct DataType< ::cs1567p1::MakeNewMazeRequest>
{
  static const char* value()
  {
    return DataType< ::cs1567p1::MakeNewMaze >::value();
  }
  static const char* value(const ::cs1567p1::MakeNewMazeRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::cs1567p1::MakeNewMazeResponse> should match 
// service_traits::MD5Sum< ::cs1567p1::MakeNewMaze > 
template<>
struct MD5Sum< ::cs1567p1::MakeNewMazeResponse>
{
  static const char* value()
  {
    return MD5Sum< ::cs1567p1::MakeNewMaze >::value();
  }
  static const char* value(const ::cs1567p1::MakeNewMazeResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::cs1567p1::MakeNewMazeResponse> should match 
// service_traits::DataType< ::cs1567p1::MakeNewMaze > 
template<>
struct DataType< ::cs1567p1::MakeNewMazeResponse>
{
  static const char* value()
  {
    return DataType< ::cs1567p1::MakeNewMaze >::value();
  }
  static const char* value(const ::cs1567p1::MakeNewMazeResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // CS1567P1_MESSAGE_MAKENEWMAZE_H
