//
// AddVector.hpp for AddVector in /home/degouv_a/Code/c++/Linscii/toro
//
// Made by alois de gouvello
// Login   <degouv_a@epitech.net>
//
// Started on  Mon Jan 27 20:25:43 2014 alois de gouvello
// Last update Mon Jan 27 21:16:03 2014 alois de gouvello
//

#ifndef ADDVECTOR_HPP_
# define ADDVECTOR_HPP_

# include <algorithm>
# include <vector>

/*
** This code is based on the snippet of Kirill V. Lyadvinsky. Thanks to him !
** I added some template and overloading to simplify.
** Hope this can help. Cheers.
*/

template <typename T>
std::vector<T> operator+(const std::vector<T> &A, const std::vector<T> &B)
{
  std::vector<T> AB;
  AB.reserve( A.size() + B.size() );		// preallocate memory
  AB.insert( AB.end(), A.begin(), A.end() );	// add A;
  AB.insert( AB.end(), B.begin(), B.end() );	// add B;
  return AB;
}

template <typename T>
std::vector<T> &operator+=(std::vector<T> &A, const std::vector<T> &B)
{
  A.reserve( A.size() + B.size() );		// preallocate memory without erase original data
  A.insert( A.end(), B.begin(), B.end() );	// add B;
  return A;					// here A could be named AB
}

#endif /* !ADDVECTOR_HPP_ */
