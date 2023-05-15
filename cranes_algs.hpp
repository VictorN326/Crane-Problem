///////////////////////////////////////////////////////////////////////////////
// cranes_algs.hpp
//
// Algorithms that solve the crane unloading problem.
//
// All of the TODO sections for this project reside in this file.
//
// This file builds on crane_types.hpp, so you should familiarize yourself
// with that file before working on this file.
//
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cassert>
#include <math.h>

#include "cranes_types.hpp"

namespace cranes
{

  // Solve the crane unloading problem for the given grid, using an exhaustive
  // optimization algorithm.
  //
  // This algorithm is expected to run in exponential time, so the grid's
  // width+height must be small enough to fit in a 64-bit int; this is enforced
  // with an assertion.
  //
  // The grid must be non-empty.
  path crane_unloading_exhaustive(const grid &setting)
  {

    // grid must be non-empty.
    assert(setting.rows() > 0);
    assert(setting.columns() > 0);

    // Compute maximum path length, and check that it is legal.
    const size_t max_steps = setting.rows() + setting.columns() - 2;
    assert(max_steps < 64);

    // TODO: implement the exhaustive search algorithm, then delete this
    // comment.
    path best(setting);
    for (size_t steps = 0; steps <= max_steps; steps++)
    {
      for (uint64_t j = 0; j <= pow(2, steps) - 1; j++)
      {
        path candidate(setting);
        bool isValid = true;
        for (uint64_t k = 0; k <= steps - 1; k++)
        {
          step_direction Direction;
          int bits = (j >> k) & 1;
          // STEP_DIRECTION EAST AND SOUTH, is_step_valid, add_step from crane_types.hpp file
          // Direction can either go east our south
          if (bits == 1)
          {
            Direction = STEP_DIRECTION_EAST;
          }
          else
          {
            Direction = STEP_DIRECTION_SOUTH;
          }
          if (candidate.is_step_valid(Direction))
          {
            candidate.add_step(Direction);
          }
          else
          {
            isValid = false;
            break;
          }
        }

        if (isValid)
        {
          if (candidate.total_cranes() > best.total_cranes())
          {
            best = candidate;
          }
        }
      }
    }

    return best;
  }

  // Solve the crane unloading problem for the given grid, using a dynamic
  // programming algorithm.
  //
  // The grid must be non-empty.
  // path crane_unloading_dyn_prog(const grid& setting) {
  path crane_unloading_dyn_prog(const grid &setting)
  {

    // grid must be non-empty.
    assert(setting.rows() > 0);
    assert(setting.columns() > 0);

    using cell_type = std::optional<path>;

    std::vector<std::vector<cell_type>> A(setting.rows(),
                                          std::vector<cell_type>(setting.columns()));

    A[0][0] = path(setting);
    assert(A[0][0].has_value());

    for (coordinate row = 0; row < setting.rows(); ++row)
    {
      for (coordinate column = 0; column < setting.columns(); ++column)
      {

        if (setting.get(row, column) == CELL_BUILDING)
        {
          A[row][column].reset();
          continue;
        }

        cell_type from_above = std::nullopt;
        cell_type from_left = std::nullopt;

        // TODO: implement the dynamic programming algorithm, then delete this comment.

        if (row > 0 && A[row - 1][column].has_value())
        {
          from_above = A[row - 1][column];
          if (from_above->is_step_valid(STEP_DIRECTION_SOUTH))
          {
            from_above->add_step(STEP_DIRECTION_SOUTH);
          }
        }

        if (column > 0 && A[row][column - 1].has_value())
        {
          from_left = A[row][column - 1];
          if (from_left->is_step_valid(STEP_DIRECTION_EAST))
          {
            from_left->add_step(STEP_DIRECTION_EAST);
          }
        }
        if (from_above.has_value() && from_left.has_value())
        {
          if (from_above->total_cranes() > from_left->total_cranes())
          {
            A[row][column] = from_above;
          }
          else
          {
            A[row][column] = from_left;
          }
        }

        else if (from_left.has_value())
        {
          A[row][column] = from_left;
        }

        else if (from_above.has_value())
        {
          A[row][column] = from_above;
        }
      }
    }
    cell_type *best = &(A[0][0]);
    assert(best->has_value());
    //this algorithm has O(n)
    for (coordinate row = 0; row < setting.rows(); ++row)
    {
      for (coordinate column = 0; column < setting.columns(); ++column)
      {
        if (A[row][column].has_value() && A[row][column]->total_cranes() > (*best)->total_cranes())
        {
          best = &(A[row][column]);
        }
      }
    }

    assert(best->has_value());
    //  //   std::cout << "total cranes" << (**best).total_cranes() << std::endl;

    return **best;
  }

}
