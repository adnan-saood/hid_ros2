// Copyright 2025 Adnan Saood
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

//

#ifndef SIGNAL_COMMAND_CONTROLLER__VISIBILITY_CONTROL_H_
#define SIGNAL_COMMAND_CONTROLLER__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SIGNAL_COMMAND_CONTROLLER_EXPORT __attribute__ ((dllexport))
    #define SIGNAL_COMMAND_CONTROLLER_IMPORT __attribute__ ((dllimport))
  #else
    #define SIGNAL_COMMAND_CONTROLLER_EXPORT __declspec(dllexport)
    #define SIGNAL_COMMAND_CONTROLLER_IMPORT __declspec(dllimport)
  #endif
  #ifdef SIGNAL_COMMAND_CONTROLLER_BUILDING_DLL
    #define SIGNAL_COMMAND_CONTROLLER_PUBLIC SIGNAL_COMMAND_CONTROLLER_EXPORT
  #else
    #define SIGNAL_COMMAND_CONTROLLER_PUBLIC SIGNAL_COMMAND_CONTROLLER_IMPORT
  #endif
  #define SIGNAL_COMMAND_CONTROLLER_PUBLIC_TYPE SIGNAL_COMMAND_CONTROLLER_PUBLIC
  #define SIGNAL_COMMAND_CONTROLLER_LOCAL
#else
  #define SIGNAL_COMMAND_CONTROLLER_EXPORT __attribute__ ((visibility("default")))
  #define SIGNAL_COMMAND_CONTROLLER_IMPORT
  #if __GNUC__ >= 4
    #define SIGNAL_COMMAND_CONTROLLER_PUBLIC __attribute__ ((visibility("default")))
    #define SIGNAL_COMMAND_CONTROLLER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SIGNAL_COMMAND_CONTROLLER_PUBLIC
    #define SIGNAL_COMMAND_CONTROLLER_LOCAL
  #endif
  #define SIGNAL_COMMAND_CONTROLLER_PUBLIC_TYPE
#endif

#endif  // SIGNAL_COMMAND_CONTROLLER__VISIBILITY_CONTROL_H_
