#pragma once

#include <ros/ros.h>

#include <iostream>
#include <memory>
#include "structures/Edge.hpp"

class Trace {
 private:
  struct Connection {
    const size_t edgeInd;
    const std::shared_ptr<Connection> before;
    const size_t size;
    const float heur;

    Connection(const size_t &edgeInd, const float &heur, std::shared_ptr<Connection> before);

    bool containsEdge(const size_t &_edgeInd) const;

    // Useful to print a Trace
    friend std::ostream &operator<<(std::ostream &os, const Connection &conn) {
      if (conn.before != nullptr) {
        os << *(conn.before);
      }
      return os << conn.edgeInd << " -> ";
    }
  };

  // ATTRIBUTE (only)
  std::shared_ptr<Connection> p;

  // CONSTRUCTORS
  Trace(std::shared_ptr<Connection> p);

 public:
  Trace();

  Trace(const size_t &edgeInd, const float &heur = 0.0);

  void addEdge(const size_t &edgeInd, const float &heur = 0.0);

  bool empty() const;

  size_t size() const;

  Trace before() const;

  Trace first() const;

  // CANT RUN IF NOT SURE THE CONE IS IN THIS
  Trace getTraceByEdgeInd(const size_t &edgeInd);

  const size_t &edgeInd() const;

  const float &heur() const;

  float sumHeur() const;

  bool containsEdge(const size_t &edgeInd) const;

  void clear();

  // Useful to print a Trace
  friend std::ostream &operator<<(std::ostream &os, const Trace &trace);
};