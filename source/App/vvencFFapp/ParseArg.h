/* -----------------------------------------------------------------------------
Software Copyright License for the Fraunhofer Software Library VVenc

(c) Copyright (2019-2020) Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V. 

1.    INTRODUCTION

The Fraunhofer Software Library VVenc (“Fraunhofer Versatile Video Encoding Library”) is software that implements (parts of) the Versatile Video Coding Standard - ITU-T H.266 | MPEG-I - Part 3 (ISO/IEC 23090-3) and related technology. 
The standard contains Fraunhofer patents as well as third-party patents. Patent licenses from third party standard patent right holders may be required for using the Fraunhofer Versatile Video Encoding Library. It is in your responsibility to obtain those if necessary. 

The Fraunhofer Versatile Video Encoding Library which mean any source code provided by Fraunhofer are made available under this software copyright license. 
It is based on the official ITU/ISO/IEC VVC Test Model (VTM) reference software whose copyright holders are indicated in the copyright notices of its source files. The VVC Test Model (VTM) reference software is licensed under the 3-Clause BSD License and therefore not subject of this software copyright license.

2.    COPYRIGHT LICENSE

Internal use of the Fraunhofer Versatile Video Encoding Library, in source and binary forms, with or without modification, is permitted without payment of copyright license fees for non-commercial purposes of evaluation, testing and academic research. 

No right or license, express or implied, is granted to any part of the Fraunhofer Versatile Video Encoding Library except and solely to the extent as expressly set forth herein. Any commercial use or exploitation of the Fraunhofer Versatile Video Encoding Library and/or any modifications thereto under this license are prohibited.

For any other use of the Fraunhofer Versatile Video Encoding Library than permitted by this software copyright license You need another license from Fraunhofer. In such case please contact Fraunhofer under the CONTACT INFORMATION below.

3.    LIMITED PATENT LICENSE

As mentioned under 1. Fraunhofer patents are implemented by the Fraunhofer Versatile Video Encoding Library. If You use the Fraunhofer Versatile Video Encoding Library in Germany, the use of those Fraunhofer patents for purposes of testing, evaluating and research and development is permitted within the statutory limitations of German patent law. However, if You use the Fraunhofer Versatile Video Encoding Library in a country where the use for research and development purposes is not permitted without a license, you must obtain an appropriate license from Fraunhofer. It is Your responsibility to check the legal requirements for any use of applicable patents.    

Fraunhofer provides no warranty of patent non-infringement with respect to the Fraunhofer Versatile Video Encoding Library.


4.    DISCLAIMER

The Fraunhofer Versatile Video Encoding Library is provided by Fraunhofer "AS IS" and WITHOUT ANY EXPRESS OR IMPLIED WARRANTIES, including but not limited to the implied warranties fitness for a particular purpose. IN NO EVENT SHALL FRAUNHOFER BE LIABLE for any direct, indirect, incidental, special, exemplary, or consequential damages, including but not limited to procurement of substitute goods or services; loss of use, data, or profits, or business interruption, however caused and on any theory of liability, whether in contract, strict liability, or tort (including negligence), arising in any way out of the use of the Fraunhofer Versatile Video Encoding Library, even if advised of the possibility of such damage.

5.    CONTACT INFORMATION

Fraunhofer Heinrich Hertz Institute
Attention: Video Coding & Analytics Department
Einsteinufer 37
10587 Berlin, Germany
www.hhi.fraunhofer.de/vvc
vvc@hhi.fraunhofer.de
----------------------------------------------------------------------------- */
#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <list>
#include <map>

//! \ingroup Interface
//! \{

namespace VVCEncoderFFApp {

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

    struct ErrorReporter
    {
      ErrorReporter() : is_errored(0) {}
      virtual ~ErrorReporter() {}
      virtual std::ostream& error(const std::string& where);
      virtual std::ostream& warn(const std::string& where);
      bool is_errored;
    };

    extern ErrorReporter default_error_reporter;

    struct SilentReporter : ErrorReporter
    {
      SilentReporter() { }
      virtual ~SilentReporter() { }
      virtual std::ostream& error( const std::string& where ) { return dest; }
      virtual std::ostream& warn( const std::string& where ) { return dest; }
      std::stringstream dest;
    };

    void doHelp(std::ostream& out, Options& opts, unsigned columns  = 120);
    std::list<const char*> scanArgv(Options& opts, unsigned argc, const char* argv[], ErrorReporter& error_reporter = default_error_reporter);
    void setDefaults(Options& opts);
    void parseConfigFile(Options& opts, const std::string& filename, ErrorReporter& error_reporter = default_error_reporter);

    /** OptionBase: Virtual base class for storing information relating to a
     * specific option This base class describes common elements.  Type specific
     * information should be stored in a derived class. */
    struct OptionBase
    {
      OptionBase(const std::string& name, const std::string& desc)
      : opt_string(name), opt_desc(desc)
      {};

      virtual ~OptionBase() {}

      /* parse argument arg, to obtain a value for the option */
      virtual void parse(const std::string& arg, ErrorReporter&) = 0;
      /* set the argument to the default value */
      virtual void setDefault() = 0;
      virtual const std::string getDefault() { return std::string(); }

      std::string opt_string;
      std::string opt_desc;
    };

    /** Type specific option storage */
    template<typename T>
    struct Option : public OptionBase
    {
      Option(const std::string& name, T& storage, T default_val, const std::string& desc)
      : OptionBase(name, desc), opt_storage(storage), opt_default_val(default_val)
      {}

      void parse(const std::string& arg, ErrorReporter&);

      void setDefault()
      {
        opt_storage = opt_default_val;
      }
      virtual const std::string getDefault(); 

      T& opt_storage;
      T opt_default_val;
    };

    template<typename T>
    inline 
    const std::string Option<T>::getDefault() 
    { 
      std::ostringstream oss;
      oss << " [" << opt_default_val << "] ";
      return oss.str(); 
    }

    /* Generic parsing */
    template<typename T>
    inline void
    Option<T>::parse(const std::string& arg, ErrorReporter&)
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

    /* string parsing is specialized -- copy the whole string, not just the
     * first word */
    template<>
    inline void
    Option<std::string>::parse(const std::string& arg, ErrorReporter&)
    {
      opt_storage = arg;
    }

    /** Option class for argument handling using a user provided function */
    struct OptionFunc : public OptionBase
    {
      typedef void (Func)(Options&, const std::string&, ErrorReporter&);

      OptionFunc(const std::string& name, Options& parent_, Func *func_, const std::string& desc)
      : OptionBase(name, desc), parent(parent_), func(func_)
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
    struct Options
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
    };

    /* Class with templated overloaded operator(), for use by Options::addOptions() */
    class OptionSpecific
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
      operator()(const std::string& name, T& storage, T default_val, const std::string& desc = "")
      {
        parent.addOption(new Option<T>(name, storage, default_val, desc));
        return *this;
      }

      /**
       * Add option described by name to the parent Options list,
       *   without separate default value
       */
      template<typename T>
      OptionSpecific&
      operator()(const std::string& name, T& storage, const std::string& desc = "")
      {
        parent.addOption(new Option<T>(name, storage, storage, desc));
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

