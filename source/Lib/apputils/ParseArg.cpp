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

Copyright (c) 2019-2021, Fraunhofer-Gesellschaft zur Förderung der angewandten Forschung e.V.
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


#include "apputils/ParseArg.h"

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <list>
#include <map>
#include <algorithm>
#include <regex>
#include <cctype>

//! \ingroup Interface
//! \{

namespace apputils {

namespace df
{
  namespace program_options_lite
  {
    ErrorReporter default_error_reporter;

    std::ostream& ErrorReporter::error(const std::string& where)
    {
      is_errored = 1;
      outstr << where << " error: ";
      return outstr;
    }

    std::ostream& ErrorReporter::warn(const std::string& where)
    {
      is_warning = 1;
      outstr << where << " warning: ";
      return outstr;
    }

    Options::~Options()
    {
      for(Options::NamesPtrList::iterator it = opt_list.begin(); it != opt_list.end(); it++)
      {
        delete *it;
      }
    }

    void Options::addOption(OptionBase *opt)
    {
      Names* names = new Names();
      names->opt = opt;
      std::string& opt_string = opt->opt_string;

      if( useLowerNamesOnly )
      {
        std::transform( opt_string.begin(), opt_string.end(), opt_string.begin(), ::tolower );
      }

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

          std::string optLongLower = opt_name;
          std::transform( optLongLower.begin(), optLongLower.end(), optLongLower.begin(), ::tolower );
          opt_long_map[optLongLower].push_back(names);
        }
        opt_start += opt_end + 1;
      }

      if( !subSections_list.empty() )
      {
        if( curSubSection.empty() ){ curSubSection = subSections_list.back(); }
        sub_section_namelist_map[curSubSection].push_back(opt_string);
      }

      opt_list.push_back(names);
    }

    int Options::setSubSection(std::string subSection)
    {
      curSubSection = subSection;
      for( auto s : subSections_list )
      {
        if( s == subSection )
        {
          return -1;
        }
      }
      subSections_list.push_back( subSection );
      return 0;
    }

    /* Helper method to initiate adding options to Options */
    OptionSpecific Options::addOptions()
    {
      if( subSections_list.empty()){ subSections_list.push_back( "__$PLACEHOLDER$__" ); } // add dummy section if nothing is given
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
      out << " [" << entry.opt->getDefault() << "] ";
    }

    static void doPrintHelpEntry( std::ostream& out, const Options::Names& entry, unsigned desc_width, unsigned opt_width, unsigned pad_short = 0 )
    {
      std::ostringstream line(std::ios_base::out);
      line << "  ";
      doHelpOpt(line, entry, pad_short);

      const std::string& opt_desc = entry.opt->opt_desc;
      if (opt_desc.empty())
      {
        /* no help text: output option, skip further processing */
        out << line.str() << std::endl;
        return;
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

    /* prints a formated config entry */
    static void printFormattedConfigEntry( std::ostream& out, const Options::Names& entry, unsigned desc_width, unsigned opt_width, unsigned opt_value_width )
    {
      // prints a config entry. format:
      // option       : vaue        # help  [default]

      std::ostringstream line(std::ios_base::out);

      if (!entry.opt_long.empty())
      {
        line << entry.opt_long.front();

        for( size_t s = 0 ; s < (opt_width - entry.opt_long.front().length()) ; ++s )
        {
          line << ' ' ;
        }

        line << ": " << entry.opt->getValue();
      }
      else
      {
        return;
      }

      const std::string& opt_desc = entry.opt->opt_desc;
      if (opt_desc.empty())
      {
        /* no help text: output option, skip further processing */
        out << line.str() << std::endl;
        return;
      }

      size_t currlength = size_t(line.tellp());
      if (currlength > opt_value_width)
      {
        /* if option text is too long (and would collide with the
         * help text, split onto next line */
        line << std::endl;
        currlength = 0;
      }
      /* split up the help text, taking into account new lines,
       *   (add opt_value_width of padding to each new line) */
      for (size_t newline_pos = 0, cur_pos = 0; cur_pos != std::string::npos; currlength = 0)
      {
        /* print any required padding space for vertical alignment */

        for( size_t num_sp = 0 ; num_sp < opt_value_width - currlength ; ++num_sp )
        {
          line << ' ' ;
        }
        newline_pos = opt_desc.find_first_of('\n', newline_pos);
        if (newline_pos != std::string::npos)
        {
          /* newline found, print substring (newline needn't be stripped) */
          newline_pos++;
          line << " # ";
          line << opt_desc.substr(cur_pos, newline_pos - cur_pos);
          cur_pos = newline_pos;
          continue;
        }
        if (cur_pos + desc_width > opt_desc.size())
        {
          /* no need to wrap text, remainder is less than avaliable width */
          line << " # ";
          line << opt_desc.substr(cur_pos);
          line << " [" << entry.opt->getDefault( ) << "] ";
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
        line << " # ";
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

      unsigned opt_width = std::min(max_width+2, 32u + pad_short) + 2;
      unsigned desc_width = columns - opt_width;

      /* second pass: write out formatted option and help text.
       *  - align start of help text to start at opt_width
       *  - if the option text is longer than opt_width, place the help
       *    text at opt_width on the next line.
       */
      if( opts.subSections_list.empty())
      {
        for (const auto& opt: opts.opt_list)
        {
          doPrintHelpEntry( out, *opt, desc_width, opt_width, pad_short );
        }
        return;
      }

      for (const auto& section: opts.subSections_list)
      {
        if( section != "__$PLACEHOLDER$__")  // print sub section name (if not dummy section)
        {
          out << std::endl << "#======== " << section << " ================" << std::endl;
        }

        Options::SubSectionNamesListMap::const_iterator itSectionMap = opts.sub_section_namelist_map.find(section);  // get list of options in subsection
        if (itSectionMap != opts.sub_section_namelist_map.end())
        {
          for( auto & s : itSectionMap->second ) // iterate over options in subsections and find/print entry in opts list
          {
            for(Options::NamesPtrList::const_iterator itopt = opts.opt_list.begin(); itopt != opts.opt_list.end(); itopt++)
            {
              if( (*itopt)->opt->opt_string == s )  // names are equal
              {
                doPrintHelpEntry( out, **itopt, desc_width, opt_width, pad_short );
                break;
               }
            }
          }
        }
      }
    }

    /* prints a formated configuration of Options into a ostream */
    void saveConfig(std::ostream& out, Options& opts, std::list<std::string> ignoreParamLst, unsigned columns )
    {
      /* first pass: work out the longest option name */
      unsigned max_width_optname = 0;
      for(Options::NamesPtrList::iterator it = opts.opt_list.begin(); it != opts.opt_list.end(); it++)
      {
        if (!(**it).opt_long.empty())
        {
          unsigned w = (unsigned)(**it).opt_long.front().size();
          max_width_optname = std::max(max_width_optname, w);
        }
      }

      /* second pass: work out the longest value*/
      unsigned max_width_value = 0;
      for(Options::NamesPtrList::iterator it = opts.opt_list.begin(); it != opts.opt_list.end(); it++)
      {
        std::string value = (**it).opt->getValue();
        max_width_value = std::max(max_width_value, (unsigned) value.length() );
      }

      unsigned max_width_opt_value = max_width_optname + max_width_value +3; // max size of option + " : " + value
      unsigned desc_width = columns - max_width_opt_value;                   // max. size for description

      /* 3rd pass: write out formatted option + current value +  help text + default value.
       * format is:
       * option       : vaue        # help  [default]
       *  - if the option text is longer than opt_width, place the help
       *    text at opt_width on the next line.
       */
      if( opts.subSections_list.empty())
      {
        for (const auto& opt: opts.opt_list)
        {
          bool ignore = ignoreParamLst.end() != std::find(ignoreParamLst.begin(), ignoreParamLst.end(), opt->opt->opt_string);
          if( !ignore)
          {
            printFormattedConfigEntry( out, *opt, desc_width, max_width_optname, max_width_opt_value );
          }
        }
        return;
      }

      // iterate over all subsections and print all entries for each subsection
      for (const auto& section: opts.subSections_list)
      {
        if( section != "__$PLACEHOLDER$__")  // print sub section name (if not dummy section)
        {
          out << std::endl << "#======== " << section << " ================" << std::endl;
        }

        Options::SubSectionNamesListMap::const_iterator itSectionMap = opts.sub_section_namelist_map.find(section);  // get list of options in subsection
        if (itSectionMap != opts.sub_section_namelist_map.end())
        {
          for( auto & s : itSectionMap->second ) // iterate over options in subsections and find/print entry in opts list
          {
            for(Options::NamesPtrList::const_iterator itopt = opts.opt_list.begin(); itopt != opts.opt_list.end(); itopt++)
            {
              if( (*itopt)->opt->opt_string == s )  // names are equal
              {
                bool ignore = ignoreParamLst.end() != std::find(ignoreParamLst.begin(), ignoreParamLst.end(), (*itopt)->opt->opt_string);
                if( !ignore)
                {
                  printFormattedConfigEntry( out, **itopt, desc_width, max_width_optname, max_width_opt_value );
                }

                break;
              }
            }
          }
        }
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
        std::string optLongLower = name;
        std::transform( optLongLower.begin(), optLongLower.end(), optLongLower.begin(), ::tolower );

        opt_it = opts.opt_long_map.find(optLongLower);
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

      std::transform( option.begin(), option.end(), option.begin(), ::tolower ); // compare option always in lower case

      unsigned extra_argc_consumed = 0;
      if (arg_opt_sep == std::string::npos)
      {
        // check if we have an argument 
        if( argc > 1)
        {
          std::string val(argv[1]);
          size_t val_sep = val.find_first_of('-');
          //check if either have no - or the parameter is a number 
          if( 0 != val_sep || std::regex_match( val, std::regex( ( "((\\+|-)?[[:digit:]]+)(\\.(([[:digit:]]+)?))?" ) ) ) ) 
          {
            extra_argc_consumed++;
            /* argument occurs after option_sep */
            storePair(true, false, option, val);
            return extra_argc_consumed;
          }
          else if( val_sep == 0 && val.size() == 1 )
          {
            extra_argc_consumed++;
            /* argument occurs after option_sep */
            storePair(true, false, option, val);
            return extra_argc_consumed;
          }
        }

        /* no argument found => argument in argv[1] (maybe) */
        /* xxx, need to handle case where option isn't required */
        if(!storePair(true, false, option, ""))
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
        storePair(false, true, option, std::string(""));
        return 1;
      }

      std::string argNext = argv[1];
      if( !argNext.empty() && argNext[0] == '-' )
      {
        // check if bool switch and check if next param is not an option
        if( argNext.size() > 1 )
        {
          if( argNext[1] == '-' ) // is long option --
          {
            storePair(false, true, option, std::string(""));
            return 0;
          }

          // check if argv is an digit number
          if( !std::isdigit(argNext[1]) )
          {
            storePair(false, true, option, std::string(""));
            return 0;
          }
        }
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

      for(unsigned i = 0; i < argc; i++)
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
        error_reporter.warn(where()) << "no value found for option " << option << "\n";
        return;
      }

      // reset empty strings
      if( value == "\"\"" || value == "''")
      {
        value.clear();
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

