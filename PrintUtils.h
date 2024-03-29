/*
 *  Created by Thayne Walker.
 *  Copyright (c) Thayne Walker 2019 All rights reserved.
 *
 * This file is part of HOG2.
 *
 * HOG2 is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG2 is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */
#ifndef PRINT_UTILS__
#define PRINT_UTILS__

#include <vector>
#include <list>
#include <array>
#include <deque>
#include <set>
#include <utility>
#include <iostream>

#include <type_traits>
#include <typeinfo>
#ifndef _MSC_VER
#   include <cxxabi.h>
#endif
#include <memory>
#include <string>
#include <cstdlib>

// Call as type_name<decltype(x)>() to print type
template <class T>
std::string
type_name()
{
    typedef typename std::remove_reference<T>::type TR;
    std::unique_ptr<char, void(*)(void*)> own
           (
#ifndef _MSC_VER
                abi::__cxa_demangle(typeid(TR).name(), nullptr,
                                           nullptr, nullptr),
#else
                nullptr,
#endif
                std::free
           );
    std::string r = own != nullptr ? own.get() : typeid(TR).name();
    if (std::is_const<TR>::value)
        r += " const";
    if (std::is_volatile<TR>::value)
        r += " volatile";
    if (std::is_lvalue_reference<T>::value)
        r += "&";
    else if (std::is_rvalue_reference<T>::value)
        r += "&&";
    return r;
}

// Call with pure type
// Call as type_name<X>() to print type
template <class T>
std::string
raw_type_name()
{
    std::unique_ptr<char, void(*)(void*)> own
           (
#ifndef _MSC_VER
                abi::__cxa_demangle(typeid(T).name(), nullptr,
                                           nullptr, nullptr),
#else
                nullptr,
#endif
                std::free
           );
    std::string r = own != nullptr ? own.get() : typeid(T).name();
    if (std::is_const<T>::value)
        r += " const";
    if (std::is_volatile<T>::value)
        r += " volatile";
    if (std::is_lvalue_reference<T>::value)
        r += "&";
    else if (std::is_rvalue_reference<T>::value)
        r += "&&";
    return r;
}


template <typename T>
inline std::ostream& operator <<(std::ostream & out, std::deque<T> const& v){
  if(v.empty()){
    out<<"[]";
    return out;
  }
  out<<"["<<*v.cbegin();
  for(auto e(v.cbegin()+1); e!=v.cend(); ++e)
    out << "," << *e;
  out<<"]";
  return out;
}

template <typename T, std::size_t V>
inline std::ostream& operator <<(std::ostream & out, std::array<T,V> const& v){
  if(v.empty()){
    out<<"[]";
    return out;
  }
  out<<"["<<*v.cbegin();
  for(auto e(v.cbegin()+1); e!=v.cend(); ++e)
    out << "," << *e;
  out<<"]";
  return out;
}

template <typename T>
inline std::ostream& operator <<(std::ostream & out, std::vector<T> const& v){
  if(v.empty()){
    out<<"[]";
    return out;
  }
  out<<"["<<*v.cbegin();
  for(auto e(v.cbegin()+1); e!=v.cend(); ++e)
    out << "," << *e;
  out<<"]";
  return out;
}

template <typename T>
inline std::ostream& operator <<(std::ostream & out, std::list<T> const& v){
  if(v.empty()){
    out<<"[]";
    return out;
  }
  auto e(v.cbegin());
  out<<"["<<*e;
  ++e;
  for(; e!=v.cend(); ++e)
    out << "," << *e;
  out<<"]";
  return out;
}

template <typename T>
inline std::ostream& operator <<(std::ostream & out, std::set<T> const& v){
  if(v.empty()){
    out<<"{}";
    return out;
  }
  auto e(v.cbegin());
  out<<"{"<<*e++;
  for(; e!=v.cend(); ++e)
    out << "," << *e;
  out<<"}";
  return out;
}

template <typename T, typename U>
inline std::ostream& operator <<(std::ostream & out, std::pair<T,U> const& v){
  out<<"("<<v.first << "," << v.second << ")";
  return out;
}

#endif
