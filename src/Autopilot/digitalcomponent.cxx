// digitalcomponent.cxx - Base class for digital autopilot components
//
// Written by Torsten Dreyer
// Based heavily on work created by Curtis Olson, started January 2004.
//
// Copyright (C) 2004  Curtis L. Olson  - http://www.flightgear.org/~curt
// Copyright (C) 2010  Torsten Dreyer - Torsten (at) t3r (dot) de
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//

#include "digitalcomponent.hxx"
#include <Main/fg_props.hxx>

#include <simgear/misc/strutils.hxx>

using std::string;
using namespace FGXMLAutopilot;

DigitalComponent::DigitalComponent() :
  _inverted(false)
{
}

bool DigitalComponent::InputMap::get_value( const std::string & name ) const
{
  // can't use map::operator[] here since it's not const
  const_iterator __i = lower_bound( name );
  if (__i == end() || key_comp()(name, (*__i).first))
    return false; // does not exist, return false

  return (*__i).second->test();
}

/*
  <input>
    <name>Foo</name>
    <condition>
     <and>...</and>
    </condition>
  </input>
  <output>
    <name>Bar</name>
    <property>/foo/bar</property>
    <inverted>true</inverted>
  </output>
  <output>/some/property</output>
*/
bool DigitalComponent::configure( SGPropertyNode& cfg_node,
                                  const std::string& cfg_name,
                                  SGPropertyNode& prop_root )
{
  if (cfg_name == "input") {
    SGPropertyNode_ptr nameNode = cfg_node.getNode("name");
    string name;
    if( nameNode != NULL ) {
      name = nameNode->getStringValue();
    } else {
      std::ostringstream buf;
      buf << "Input" << _input.size();
      name = buf.str();
    }
    _input[name] = sgReadCondition(&prop_root, &cfg_node);
    return true;
  } 

  if (cfg_name == "output") {
    SGPropertyNode_ptr n = cfg_node.getNode("name");
    string name;
    if( n != NULL ) {
      name = n->getStringValue();
    } else {
      std::ostringstream buf;
      buf << "Output" << _output.size();
      name = buf.str();
    }

    DigitalOutput_ptr o = new DigitalOutput();
    _output[name] = o;

    if( (n = cfg_node.getNode("inverted")) != NULL )
      o->setInverted( n->getBoolValue() );

    if( (n = cfg_node.getNode("property")) != NULL ) {
        const auto trimmed = simgear::strutils::strip(n->getStringValue());
        o->setProperty( prop_root.getNode(trimmed, true) );
    }

    if( cfg_node.nChildren() == 0 ) {
        const auto trimmed = simgear::strutils::strip(cfg_node.getStringValue());
        o->setProperty( prop_root.getNode(trimmed, true) );
    }
      
    return true;
  } 

  if (cfg_name == "inverted") {
    _inverted = cfg_node.getBoolValue();
    return true;
  }
  
  return Component::configure(cfg_node, cfg_name, prop_root);
}
