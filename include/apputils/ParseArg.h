/* -----------------------------------------------------------------------------
The copyright in this software is being made available under the BSD
License, included below. No patent rights, trademark rights and/or 
other Intellectual Property Rights other than the copyrights concerning 
the Software are granted under this license.

For any license concerning other Intellectual Property rights than the software,
especially patent licenses, a separate Agreement needs to be closed. 
For more information please contact:

Fraunhofer Heinrich Hertz Institute
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de

Copyright (c) 2019-2020, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
 * Neither the name of Fraunhofer nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
THE POSSIBILITY OF SUCH DAMAGE.


------------------------------------------------------------------------------------------- */
#pragma once

#include "apputils/apputilsDecl.h"

#include <iostream>
#include <sstream>
#include <string>
#include <list>
#include <map>

//! \ingroup Interface
//! \{

namespace apputils {

namespace df
{
  namespace program_options_lite
  {
    struct Options;

    struct ParseFailure : public std::exception
    {
      ParseFailure(std::string arg0, std::string val0) throw()
      : arg(arg0), val(val0)
      {}

      ~ParseFailure() throw() {};

      std::string arg;
      std::string val;

      const char* what() const throw() { return "Option Parse Failure"; }
    };

    struct APPUTILS_DECL ErrorReporter
    {
      ErrorReporter() : is_errored(0) {}
      virtual ~ErrorReporter() {}
      virtual std::ostream& error(const std::string& where);
      virtual std::ostream& warn(const std::string& where);
      bool is_errored;
    };

    extern ErrorReporter default_error_reporter;

    struct APPUTILS_DECL SilentReporter : ErrorReporter
    {
      SilentReporter() { }
      virtual ~SilentReporter() { }
      virtual std::ostream& error( const std::string& ) { return dest; }
      virtual std::ostream& warn( const std::string& ) { return dest; }
      std::stringstream dest;
    };

    void APPUTILS_DECL doHelp(std::ostream& out, Options& opts, unsigned columns  = 120);
    void APPUTILS_DECL saveConfig(std::ostream& out, Options& opts, std::list<std::string> ignoreParamLst, unsigned columns = 240 );

    std::list<const char*> APPUTILS_DECL scanArgv(Options& opts, unsigned argc, const char* argv[], ErrorReporter& error_reporter = default_error_reporter);
    void APPUTILS_DECL setDefaults(Options& opts);
    void APPUTILS_DECL parseConfigFile(Options& opts, const std::string& filename, ErrorReporter& error_reporter = default_error_reporter);

    /** OptionBase: Virtual base class for storing information relating to a
     * specific option This base class describes common elements.  Type specific
     * information should be stored in a derived class. */
    struct OptionBase
    {
      OptionBase(const std::string& name, const std::string& desc, const bool bool_switch )
      : opt_string(name), opt_desc(desc), is_bool_switch(bool_switch)
      {};

      virtual ~OptionBase() {}

      /* parse argument arg, to obtain a value for the option */
      virtual void parse(const std::string& arg, ErrorReporter&) = 0;
      /* set the argument to the default value */
      virtual void setDefault() = 0;
      virtual const std::string getDefault( ) { return std::string(); }
      virtual const std::string getValue( ) { return std::string(); }

      std::string opt_string;
      std::string opt_desc;
      bool        is_bool_switch = false;
    };

    /** Type specific option storage */
    template<typename T>
    struct Option : public OptionBase
    {
      Option(const std::string& name, T& storage, T default_val, const std::string& desc, const bool bool_switch )
      : OptionBase(name, desc, bool_switch), opt_storage(storage), opt_default_val(default_val)
      {}

      void parse(const std::string& arg, ErrorReporter&);

      void setDefault()
      {
        opt_storage = opt_default_val;
      }
      virtual const std::string getDefault( );
      virtual const std::string getValue  ( );

      T& opt_storage;
      T opt_default_val;
    };

    template<typename T>
    inline
    const std::string Option<T>::getValue( )
    {
      std::ostringstream oss;
      oss << opt_storage;
      return oss.str();
    }

    template<>
    inline
    const std::string Option<std::string>::getValue( )
    {
      std::ostringstream oss;
      if( opt_storage.empty() )
      {
        oss << "\"\"";
      }
      else
      {
        oss << opt_storage;
      }
      return oss.str();
    }

    template<typename T>
    inline 
    const std::string Option<T>::getDefault( )
    { 
      std::ostringstream oss;
      oss << opt_default_val;
      return oss.str(); 
    }

    /* Generic parsing */
    template<typename T>
    inline void
    Option<T>::parse(const std::string& arg, ErrorReporter&)
    {
      std::string param = arg;
      if( is_bool_switch )
      {
        if( arg.empty() ) { param = "1"; }
      }

      std::istringstream arg_ss (param,std::istringstream::in);
      arg_ss.exceptions(std::ios::failbit);
      try
      {
        arg_ss >> opt_storage;
      }
      catch (...)
      {
        throw ParseFailure(opt_string, param);
      }
    }

    /* string parsing is specialized -- copy the whole string, not just the
     * first word */
    template<>
    inline void
    Option<std::string>::parse(const std::string& arg, ErrorReporter&)
    {
      opt_storage = arg;
    }

    template<>
    inline void
    Option<bool>::parse(const std::string& arg, ErrorReporter&)
    {
      if( arg.empty() )
      {
        opt_storage = true;
      }
      else
      {
        std::istringstream arg_ss (arg,std::istringstream::in);
        arg_ss.exceptions(std::ios::failbit);
        try
        {
          arg_ss >> opt_storage;
        }
        catch (...)
        {
          throw ParseFailure(opt_string, arg);
        }
      }
    }

    /** Option class for argument handling using a user provided function */
    struct OptionFunc : public OptionBase
    {
      typedef void (Func)(Options&, const std::string&, ErrorReporter&);

      OptionFunc(const std::string& name, Options& parent_, Func *func_, const std::string& desc)
      : OptionBase(name, desc, false), parent(parent_), func(func_)
      {}

      void parse(const std::string& arg, ErrorReporter& error_reporter)
      {
        func(parent, arg, error_reporter);
      }

      void setDefault()
      {
        return;
      }

    private:
      Options& parent;
      Func* func;
    };

    class OptionSpecific;
    struct APPUTILS_DECL Options
    {
      ~Options();

      OptionSpecific addOptions();

      struct Names
      {
        Names() : opt(0) {};
        ~Names()
        {
          if (opt)
          {
            delete opt;
          }
        }
        std::list<std::string> opt_long;
        std::list<std::string> opt_short;
        OptionBase* opt;
      };

      void addOption(OptionBase *opt);

      typedef std::list<Names*> NamesPtrList;
      NamesPtrList opt_list;

      typedef std::map<std::string, NamesPtrList> NamesMap;
      NamesMap opt_long_map;
      NamesMap opt_short_map;

      int setSubSection(std::string subSection);

      typedef std::list<std::string> subSectionsPtrList;
      subSectionsPtrList subSections_list;
      std::string curSubSection;
      
      typedef std::map<std::string, std::list<std::string> > SubSectionNamesListMap;
      SubSectionNamesListMap sub_section_namelist_map;
    };

    /* Class with templated overloaded operator(), for use by Options::addOptions() */
    class APPUTILS_DECL OptionSpecific
    {
    public:
      OptionSpecific(Options& parent_) : parent(parent_) {}

      /**
       * Add option described by name to the parent Options list,
       *   with storage for the option's value
       *   with default_val as the default value
       *   with desc as an optional help description
       */
      template<typename T>
      OptionSpecific&
      operator()(const std::string& name, T& storage, T default_val, const std::string& desc = "", bool bool_switch = false)
      {
        parent.addOption(new Option<T>(name, storage, default_val, desc, bool_switch));
        return *this;
      }

      /**
       * Add option described by name to the parent Options list,
       *   without separate default value
       */
      template<typename T>
      OptionSpecific&
      operator()(const std::string& name, T& storage, const std::string& desc = "", bool bool_switch = false )
      {
        parent.addOption(new Option<T>(name, storage, storage, desc, bool_switch));
        return *this;
      }

      /**
       * Add option described by name to the parent Options list,
       *   with desc as an optional help description
       * instead of storing the value somewhere, a function of type
       * OptionFunc::Func is called.  It is upto this function to correctly
       * handle evaluating the option's value.
       */
      OptionSpecific&
      operator()(const std::string& name, OptionFunc::Func *func, const std::string& desc = "")
      {
        parent.addOption(new OptionFunc(name, parent, func, desc));
        return *this;
      }
    private:
      Options& parent;
    };

  } /* namespace: program_options_lite */
} /* namespace: df */

} // namespace

//! \}

