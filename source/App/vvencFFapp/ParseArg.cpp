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


#include "../vvencFFapp/ParseArg.h"

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <list>
#include <map>
#include <algorithm>

//! \ingroup Interface
//! \{

namespace VVCEncoderFFApp {

namespace df
{
  namespace program_options_lite
  {
    ErrorReporter default_error_reporter;

    std::ostream& ErrorReporter::error(const std::string& where)
    {
      is_errored = 1;
      std::cerr << where << " error: ";
      return std::cerr;
    }

    std::ostream& ErrorReporter::warn(const std::string& where)
    {
      std::cerr << where << " warning: ";
      return std::cerr;
    }

    Options::~Options()
    {
      for(Options::NamesPtrList::iterator it = opt_list.begin(); it != opt_list.end(); it++)
      {
        delete *it;
      }
    }

    void Options::initOptions( const Options& opt)
    {
      for( const auto& o : opt.opt_list )
      {
        addOption(o->opt);
      }
    }

    void Options::addOption(OptionBase *opt)
    {
      Names* names = new Names();
      names->opt = opt;
      std::string& opt_string = opt->opt_string;

      size_t opt_start = 0;
      for (size_t opt_end = 0; opt_end != std::string::npos;)
      {
        opt_end = opt_string.find_first_of(',', opt_start);
        bool force_short = 0;
        if (opt_string[opt_start] == '-')
        {
          opt_start++;
          force_short = 1;
        }
        std::string opt_name = opt_string.substr(opt_start, opt_end - opt_start);
        if (force_short || opt_name.size() == 1)
        {
          names->opt_short.push_back(opt_name);
          opt_short_map[opt_name].push_back(names);
        }
        else
        {
          names->opt_long.push_back(opt_name);
          opt_long_map[opt_name].push_back(names);
        }
        opt_start += opt_end + 1;
      }
      opt_list.push_back(names);
    }

    /* Helper method to initiate adding options to Options */
    OptionSpecific Options::addOptions()
    {
      return OptionSpecific(*this);
    }

    static void setOptions(Options::NamesPtrList& opt_list, const std::string& value, ErrorReporter& error_reporter)
    {
      /* multiple options may be registered for the same name:
       *   allow each to parse value */
      for (Options::NamesPtrList::iterator it = opt_list.begin(); it != opt_list.end(); ++it)
      {
        (*it)->opt->parse(value, error_reporter);
      }
    }

    static const char spaces[41] = "                                        ";

    /* format help text for a single option:
     * using the formatting: "-x, --long",
     * if a short/long option isn't specified, it is not printed
     */
    static void doHelpOpt(std::ostream& out, const Options::Names& entry, unsigned pad_short = 0)
    {
      pad_short = std::min(pad_short, 8u);

      if (!entry.opt_short.empty())
      {
        unsigned pad = std::max((int)pad_short - (int)entry.opt_short.front().size(), 0);
        out << "-" << entry.opt_short.front();
        if (!entry.opt_long.empty())
        {
          out << ", ";
        }
        out << &(spaces[40 - pad]);
      }
      else
      {
        out << "   ";
        out << &(spaces[40 - pad_short]);
      }

      if (!entry.opt_long.empty())
      {
        out << "--" << entry.opt_long.front();
      }
    }

    /* format the help text */
    void doHelp(std::ostream& out, Options& opts, unsigned columns)
    {
      const unsigned pad_short = 3;
      /* first pass: work out the longest option name */
      unsigned max_width = 0;
      for(Options::NamesPtrList::iterator it = opts.opt_list.begin(); it != opts.opt_list.end(); it++)
      {
        std::ostringstream line(std::ios_base::out);
        doHelpOpt(line, **it, pad_short);
        max_width = std::max(max_width, (unsigned) line.tellp());
      }

      unsigned opt_width = std::min(max_width+2, 28u + pad_short) + 2;
      unsigned desc_width = columns - opt_width;

      /* second pass: write out formatted option and help text.
       *  - align start of help text to start at opt_width
       *  - if the option text is longer than opt_width, place the help
       *    text at opt_width on the next line.
       */
      for(Options::NamesPtrList::iterator it = opts.opt_list.begin(); it != opts.opt_list.end(); it++)
      {
        std::ostringstream line(std::ios_base::out);
        line << "  ";
        doHelpOpt(line, **it, pad_short);

        const std::string& opt_desc = (*it)->opt->opt_desc;
        if (opt_desc.empty())
        {
          /* no help text: output option, skip further processing */
          out << line.str() << std::endl;
          continue;
        }
        size_t currlength = size_t(line.tellp());
        if (currlength > opt_width)
        {
          /* if option text is too long (and would collide with the
           * help text, split onto next line */
          line << std::endl;
          currlength = 0;
        }
        /* split up the help text, taking into account new lines,
         *   (add opt_width of padding to each new line) */
        for (size_t newline_pos = 0, cur_pos = 0; cur_pos != std::string::npos; currlength = 0)
        {
          /* print any required padding space for vertical alignment */
          line << &(spaces[40 - opt_width + currlength]);
          newline_pos = opt_desc.find_first_of('\n', newline_pos);
          if (newline_pos != std::string::npos)
          {
            /* newline found, print substring (newline needn't be stripped) */
            newline_pos++;
            line << opt_desc.substr(cur_pos, newline_pos - cur_pos);
            cur_pos = newline_pos;
            continue;
          }
          if (cur_pos + desc_width > opt_desc.size())
          {
            /* no need to wrap text, remainder is less than avaliable width */
            line << opt_desc.substr(cur_pos);
            break;
          }
          /* find a suitable point to split text (avoid spliting in middle of word) */
          size_t split_pos = opt_desc.find_last_of(' ', cur_pos + desc_width);
          if (split_pos != std::string::npos)
          {
            /* eat up multiple space characters */
            split_pos = opt_desc.find_last_not_of(' ', split_pos) + 1;
          }

          /* bad split if no suitable space to split at.  fall back to width */
          bool bad_split = split_pos == std::string::npos || split_pos <= cur_pos;
          if (bad_split)
          {
            split_pos = cur_pos + desc_width;
          }
          line << opt_desc.substr(cur_pos, split_pos - cur_pos);

          /* eat up any space for the start of the next line */
          if (!bad_split)
          {
            split_pos = opt_desc.find_first_not_of(' ', split_pos);
          }
          cur_pos = newline_pos = split_pos;

          if (cur_pos >= opt_desc.size())
          {
            break;
          }
          line << std::endl;
        }

        out << line.str() << std::endl;
      }
    }

    struct OptionWriter
    {
      OptionWriter(Options& rOpts, ErrorReporter& err)
      : opts(rOpts), error_reporter(err)
      {}
      virtual ~OptionWriter() {}

      virtual const std::string where() = 0;

      bool storePair(bool allow_long, bool allow_short, const std::string& name, const std::string& value);
      bool storePair(const std::string& name, const std::string& value)
      {
        return storePair(true, true, name, value);
      }

      Options& opts;
      ErrorReporter& error_reporter;
    };

    bool OptionWriter::storePair(bool allow_long, bool allow_short, const std::string& name, const std::string& value)
    {
      bool found = false;
      Options::NamesMap::iterator opt_it;
      if (allow_long)
      {
        opt_it = opts.opt_long_map.find(name);
        if (opt_it != opts.opt_long_map.end())
        {
          found = true;
        }
      }

      /* check for the short list */
      if (allow_short && !(found && allow_long))
      {
        opt_it = opts.opt_short_map.find(name);
        if (opt_it != opts.opt_short_map.end())
        {
          found = true;
        }
      }

      if (!found)
      {
        error_reporter.error(where())
          << "Unknown option `" << name << "' (value:`" << value << "')\n";
        return false;
      }

      setOptions((*opt_it).second, value, error_reporter);
      return true;
    }

    struct ArgvParser : public OptionWriter
    {
      ArgvParser(Options& rOpts, ErrorReporter& rError_reporter)
      : OptionWriter(rOpts, rError_reporter)
      {}

      const std::string where() { return "command line"; }

      unsigned parseGNU(unsigned argc, const char* argv[]);
      unsigned parseSHORT(unsigned argc, const char* argv[]);
    };

    /**
     * returns number of extra arguments consumed
     */
    unsigned ArgvParser::parseGNU(unsigned argc, const char* argv[])
    {
      /* gnu style long options can take the forms:
       *  --option=arg
       *  --option arg
       */
      std::string arg(argv[0]);
      size_t arg_opt_start = arg.find_first_not_of('-');
      size_t arg_opt_sep = arg.find_first_of('=');
      std::string option = arg.substr(arg_opt_start, arg_opt_sep - arg_opt_start);

      unsigned extra_argc_consumed = 0;
      if (arg_opt_sep == std::string::npos)
      {
        /* no argument found => argument in argv[1] (maybe) */
        /* xxx, need to handle case where option isn't required */
        if(!storePair(true, false, option, "1"))
        {
          return 0;
        }
      }
      else
      {
        /* argument occurs after option_sep */
        std::string val = arg.substr(arg_opt_sep + 1);
        storePair(true, false, option, val);
      }

      return extra_argc_consumed;
    }

    unsigned ArgvParser::parseSHORT(unsigned argc, const char* argv[])
    {
      /* short options can take the forms:
       *  --option arg
       *  -option arg
       */
      std::string arg(argv[0]);
      size_t arg_opt_start = arg.find_first_not_of('-');
      std::string option = arg.substr(arg_opt_start);
      /* lookup option */

      /* argument in argv[1] */
      /* xxx, need to handle case where option isn't required */
      if (argc == 1)
      {
        error_reporter.error(where())
          << "Not processing option `" << option << "' without argument\n";
        return 0; /* run out of argv for argument */
      }
      storePair(false, true, option, std::string(argv[1]));

      return 1;
    }

    std::list<const char*>
    scanArgv(Options& opts, unsigned argc, const char* argv[], ErrorReporter& error_reporter)
    {
      ArgvParser avp(opts, error_reporter);

      /* a list for anything that didn't get handled as an option */
      std::list<const char*> non_option_arguments;

      for(unsigned i = 1; i < argc; i++)
      {
        if (argv[i][0] != '-')
        {
          non_option_arguments.push_back(argv[i]);
          continue;
        }

        if (argv[i][1] == 0)
        {
          /* a lone single dash is an argument (usually signifying stdin) */
          non_option_arguments.push_back(argv[i]);
          continue;
        }

        if (argv[i][1] != '-')
        {
          /* handle short (single dash) options */
          i += avp.parseSHORT(argc - i, &argv[i]);
          continue;
        }

        if (argv[i][2] == 0)
        {
          /* a lone double dash ends option processing */
          while (++i < argc)
          {
            non_option_arguments.push_back(argv[i]);
          }
          break;
        }

        /* handle long (double dash) options */
        i += avp.parseGNU(argc - i, &argv[i]);
      }

      return non_option_arguments;
    }

    struct CfgStreamParser : public OptionWriter
    {
      CfgStreamParser(const std::string& rName, Options& rOpts, ErrorReporter& rError_reporter)
      : OptionWriter(rOpts, rError_reporter)
      , name(rName)
      , linenum(0)
      {}

      const std::string name;
      int linenum;
      const std::string where()
      {
        std::ostringstream os;
        os << name << ":" << linenum;
        return os.str();
      }

      void scanLine(std::string& line);
      void scanStream(std::istream& in);
    };

    void CfgStreamParser::scanLine(std::string& line)
    {
      /* strip any leading whitespace */
      size_t start = line.find_first_not_of(" \t\n\r");
      if (start == std::string::npos)
      {
        /* blank line */
        return;
      }
      if (line[start] == '#')
      {
        /* comment line */
        return;
      }
      /* look for first whitespace or ':' after the option end */
      size_t option_end = line.find_first_of(": \t\n\r",start);
      std::string option = line.substr(start, option_end - start);

      /* look for ':', eat up any whitespace first */
      start = line.find_first_not_of(" \t\n\r", option_end);
      if (start == std::string::npos)
      {
        /* error: badly formatted line */
        error_reporter.warn(where()) << "line formatting error\n";
        return;
      }
      if (line[start] != ':')
      {
        /* error: badly formatted line */
        error_reporter.warn(where()) << "line formatting error\n";
        return;
      }

      /* look for start of value std::string -- eat up any leading whitespace */
      start = line.find_first_not_of(" \t\n\r", ++start);
      if (start == std::string::npos)
      {
        /* error: badly formatted line */
        error_reporter.warn(where()) << "line formatting error\n";
        return;
      }

      /* extract the value part, which may contain embedded spaces
       * by searching for a word at a time, until we hit a comment or end of line */
      size_t value_end = start;
      do
      {
        if (line[value_end] == '#')
        {
          /* rest of line is a comment */
          value_end--;
          break;
        }
        value_end = line.find_first_of(" \t\n\r", value_end);
        /* consume any white space, incase there is another word.
         * any trailing whitespace will be removed shortly */
        value_end = line.find_first_not_of(" \t\n\r", value_end);
      } while (value_end != std::string::npos);
      /* strip any trailing space from value*/
      value_end = line.find_last_not_of(" \t\n\r", value_end);

      std::string value;
      if (value_end >= start)
      {
        value = line.substr(start, value_end +1 - start);
      }
      else
      {
        /* error: no value */
        error_reporter.warn(where()) << "no value found\n";
        return;
      }

      /* store the value in option */
      storePair(true, false, option, value);
    }

    void CfgStreamParser::scanStream(std::istream& in)
    {
      do
      {
        linenum++;
        std::string line;
        getline(in, line);
        scanLine(line);
      } while(!!in);
    }

    /* for all options in opts, set their storage to their specified
     * default value */
    void setDefaults(Options& opts)
    {
      for(Options::NamesPtrList::iterator it = opts.opt_list.begin(); it != opts.opt_list.end(); it++)
      {
        (*it)->opt->setDefault();
      }
    }

    void parseConfigFile(Options& opts, const std::string& filename, ErrorReporter& error_reporter)
    {
      std::ifstream cfgstream(filename.c_str(), std::ifstream::in);
      if (!cfgstream)
      {
        error_reporter.error(filename) << "Failed to open config file\n";
        return;
      }
      CfgStreamParser csp(filename, opts, error_reporter);
      csp.scanStream(cfgstream);
    }

  }
}

} // namespace VVCEncoderFFApp

//! \}

