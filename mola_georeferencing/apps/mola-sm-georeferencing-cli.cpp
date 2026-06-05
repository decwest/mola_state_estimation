/*               _
 _ __ ___   ___ | | __ _
| '_ ` _ \ / _ \| |/ _` | Modular Optimization framework for
| | | | | | (_) | | (_| | Localization and mApping (MOLA)
|_| |_| |_|\___/|_|\__,_| https://github.com/MOLAorg/mola

 Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
                         and individual contributors.
 SPDX-License-Identifier: GPL-3.0
 See LICENSE for full license information.
 Closed-source licenses available upon request, for this package
 alone or in combination with the complete SLAM system.
*/

#include <mola_georeferencing/simplemap_georeference.h>
#include <mp2p_icp/metricmap.h>
#include <CLI/CLI.hpp>
#include <mrpt/containers/yaml.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <fstream>

// CLI flags:

struct Cli
{
    CLI::App cmd{"mola-sm-georeferencing-cli"};

    std::string argInput;
    std::string argWriteMMInto;
    std::string argOutput;
    double      argHorz{1.0};
    bool        argHorzSet{false};
    double      argIMUGravitySigmaDeg{3.0};
    bool        argIMUGravitySigmaDegSet{false};
    std::string argPlugins;
    bool        argNoIMUGravity{false};
    std::string arg_verbosity_level{"INFO"};

    void setup()
    {
        cmd.add_option("-i,--input", argInput, "Input .simplemap file")->required();
        cmd.add_option("--write-into", argWriteMMInto,
            "An existing .mm file in which to write the georeferencing metadata");
        cmd.add_option("-o,--output", argOutput,
            "Write the obtained georeferencing metadata to a file. The format is "
            "determined by the file extension: binary gzip (`*.georef`) or YAML "
            "(`*.yaml`, `*.yml`).");
        cmd.add_option("--horizontality-sigma", argHorz,
            "For short trajectories (not >10x the GPS uncertainty), this helps to "
            "avoid degeneracy.")->each([this](const std::string&){ argHorzSet = true; });
        cmd.add_option("--imu-gravity-sigma-deg", argIMUGravitySigmaDeg,
            "IMU gravity alignment uncertainty (degrees).")
            ->each([this](const std::string&){ argIMUGravitySigmaDegSet = true; });
        cmd.add_option("-l,--load-plugins", argPlugins,
            "One or more (comma separated) *.so files to load as plugins, e.g. "
            "defining new CMetricMap classes");
        cmd.add_flag("--no-imu-gravity", argNoIMUGravity,
            "Disable using IMU acceleration data for gravity alignment "
            "(enabled by default).");
        cmd.add_option("-v,--verbosity", arg_verbosity_level,
            "Verbosity level: ERROR|WARN|INFO|DEBUG (Default: INFO)");
    }
};

static bool is_binary_georef(const std::string& fil)
{
    return mrpt::system::extractFileExtension(fil) == "georef";
}

void run_sm_georef(Cli& cli)
{
    if (!cli.argPlugins.empty())
    {
        std::string sErrs;
        bool        ok = mrpt::system::loadPluginModules(cli.argPlugins, sErrs);
        if (!ok)
        {
            std::cerr << "Errors loading plugins: " << cli.argPlugins << std::endl;
            throw std::runtime_error(sErrs.c_str());
        }
    }

    mrpt::system::COutputLogger logger;
    logger.setLoggerName("mola-sm-georeferencing-cli");
    logger.setVerbosityLevel(
        mrpt::typemeta::str2enum<mrpt::system::VerbosityLevel>(cli.arg_verbosity_level));

    const auto& filSM = cli.argInput;

    mrpt::maps::CSimpleMap sm;

    logger.logFmt(mrpt::system::LVL_INFO, "Reading simplemap from: '%s'...", filSM.c_str());

    sm.loadFromFile(filSM);

    logger.logFmt(mrpt::system::LVL_INFO, "Done read simplemap with %zu keyframes.", sm.size());

    ASSERT_(!sm.empty());

    mola::SMGeoReferencingParams p;
    p.logger = &logger;

    if (cli.argHorzSet)
    {
        p.fgParams.addHorizontalityConstraints = true;
        p.fgParams.horizontalitySigmaZ         = cli.argHorz;
    }
    // TODO: p.fgParams.minimumUncertaintyXYZ = xxx;

    if (cli.argNoIMUGravity)
    {
        p.useIMUGravityAlignment = false;
    }
    if (cli.argIMUGravitySigmaDegSet)
    {
        p.imuGravityParams.imuGravitySigmaDeg = cli.argIMUGravitySigmaDeg;
    }

    const mola::SMGeoReferencingOutput smGeo = mola::simplemap_georeference(sm, p);

    if (!smGeo.geo_ref.has_value())
    {
        std::cerr << "Georeferencing failed. No output will be generated.\n";
        return;
    }

    const auto& geo_ref = smGeo.geo_ref.value();

    std::cout << "Obtained georeferencing:\n"
              << "lat: " << geo_ref.geo_coord.lat.getAsString() << "\n"
              << "lon: " << geo_ref.geo_coord.lon.getAsString() << "\n"
              << mrpt::format(
                     "lat_lon: %.06f, %.06f\n", geo_ref.geo_coord.lat.decimal_value,
                     geo_ref.geo_coord.lon.decimal_value)
              << "h: " << geo_ref.geo_coord.height << "\n"
              << "T_enu_to_map: " << geo_ref.T_enu_to_map.asString() << "\n";

    // Warn if the user has not requested any output at all.
    if (cli.argWriteMMInto.empty() && cli.argOutput.empty())
    {
        std::cerr
            << "[mola-sm-georeferencing-cli] WARNING: Georeferencing was computed successfully "
               "but no output destination was specified.\n"
               "  Use '--write-into <map.mm>' to inject it into a metric map, or\n"
               "  '-o <map.georef|map.yaml>' to save it to a standalone file.\n"
               "  The result will be discarded.\n";
    }

    if (!cli.argWriteMMInto.empty())
    {
        mp2p_icp::metric_map_t mm;

        std::cout << "[mola-sm-georeferencing-cli] Loading mm map: '"
                  << cli.argWriteMMInto << "'..." << std::endl;

        const bool loadOk = mm.load_from_file(cli.argWriteMMInto);
        if (!loadOk)
        {
            THROW_EXCEPTION_FMT(
                "Error loading input map file: '%s'", cli.argWriteMMInto.c_str());
        }

        // overwrite metadata:
        mm.georeferencing = smGeo.geo_ref;

        // and save:
        const auto saved_ok = mm.save_to_file(cli.argWriteMMInto);
        if (!saved_ok)
        {
            std::cerr << "Error saving modified .mm file: '" << cli.argWriteMMInto
                      << "'\n";
            return;
        }

        std::cout << "[mola-sm-georeferencing-cli] Writing modified mm map: '"
                  << cli.argWriteMMInto << "'..." << std::endl;
    }

    if (!cli.argOutput.empty())
    {
        const std::string outFil = cli.argOutput;

        std::cout << "[mola-sm-georeferencing-cli] Writing georef data file: '" << outFil << "'..."
                  << std::endl;

        std::optional<mp2p_icp::metric_map_t::Georeferencing> g = smGeo.geo_ref;

        if (is_binary_georef(outFil))
        {
            // Binary gzip format (*.georef)
            mrpt::io::CFileGZOutputStream f(outFil);
            auto                          arch = mrpt::serialization::archiveFrom(f);
            arch << g;
        }
        else
        {
            // YAML format (*.yaml, *.yml, or any other extension)
            const auto    yamlData = mp2p_icp::ToYAML(g);
            std::ofstream of(outFil);
            if (!of.is_open())
            {
                THROW_EXCEPTION_FMT("Cannot open output file for writing: '%s'", outFil.c_str());
            }
            of << yamlData;
        }
    }
}

int main(int argc, char** argv)
{
    try
    {
        Cli cli;
        cli.setup();

        try
        {
            cli.cmd.parse(argc, argv);
        }
        catch (const CLI::ParseError& e)
        {
            return cli.cmd.exit(e);
        }

        run_sm_georef(cli);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what();
        return 1;
    }
    return 0;
}
