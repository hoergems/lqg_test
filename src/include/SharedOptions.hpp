/** @file SharedOptions.hpp
 *
 * Defines the SharedOptions class, which comes with additional configuration options that apply
 * to many problems.
 */
#ifndef SHAREDOPTIONS_HPP_
#define SHAREDOPTIONS_HPP_

#include "tclap/CmdLine.h"
#include "option_parser.hpp"
#include "Options.hpp"

namespace shared {
/** An expanded Options class, which comes with some additional settings that are shared by many
 * of the individual example problems.
 *
 * These extra configuration options allow for extra configuration at runtime instead of
 * compile-time.
 */
struct SharedOptions: public solver::Options {
    SharedOptions() = default;
    virtual ~SharedOptions() = default;

    /* ---------------------- Generic settings  ------------------ */
    /** The base config path - the path of the configuration files will be relative to this path. */
    std::string baseConfigPath = "";
    /** The path to the configuration file (relative to baseConfigPath). */
    std::string configPath = "";
    /** The path to the policy file. */
    std::string policyPath = "";
    /** The seed value to use for the RNG. */
    unsigned long seed = 0;
    /** A custom state to load for RNG. */
    unsigned long rngState = 0;

    /* --------------------- Simulation settings  ----------------- */
    /** The path to the log file. */
    std::string logPath = "";
    /** The maximum number of steps to simulate per run. */
    long nSimulationSteps = 0;
    /** The number of simulations to run. */
    long nRuns = 0;
    /** True iff we should load an initial policy before running the simulation. */
    bool loadInitialPolicy = false;
    /** True iff we should save the resulting policy at the end of each simulation. */
    bool savePolicy = false;
    /** True iff we should save the particles representing the belief states at the end of each simulation. */
    bool saveParticles = false;

    /* ----------------- Simulation settings: changes  ------------ */
    /** True iff there are pre-planned model changed during the simulation. */
    bool hasChanges = false;
    /** True iff the changes are dynamic. */
    bool areDynamic = false;
    /** The path to the change file (relative to baseConfigPath) */
    std::string changesPath = "";

    /* ---------- ABT settings: advanced customization  ---------- */
    /** The heuristic used for searches. */
    std::string searchHeuristic = "";
    /** The search strategy to use. */
    std::string searchStrategy = "";
    /** The recommendation strategy to use. */
    std::string recommendationStrategy = "";
    /** The function to estimate the value of a belief. */
    std::string estimator = "";
    /** The maximum distance between observations to group together; only applicable if
     * approximate observations are in use. */
    double maxObservationDistance = 0.0;

    /** Makes a parser which can parse options from config files, or from the command line,
     * into a SharedOptions instance.
     */
    static std::unique_ptr<options::OptionParser> makeParser(bool simulating,
            std::string defaultBaseConfigPath) {
        std::unique_ptr<options::OptionParser> parser = std::make_unique<options::OptionParser>(
                "TAPIR command line interface");
        addGenericOptions(parser.get(), defaultBaseConfigPath);
        addSimulationOptions(parser.get(), simulating);
        addABTOptions(parser.get());
        addProblemOptions(parser.get());
        return std::move(parser);
    }

    /** Adds generic options for this SharedOptions instance to the given parser, using the
     * given default configuration file path.
     */
    static void addGenericOptions(options::OptionParser *parser, std::string defaultBaseConfigPath) {
        parser->addOptionWithDefault<std::string>("", "baseConfigPath",
                &SharedOptions::baseConfigPath, defaultBaseConfigPath);
        parser->addValueArg("", "baseConfigPath", &SharedOptions::baseConfigPath, "", "base-path",
                "the base config path", "path");

        parser->addOptionWithDefault<std::string>("", "cfg", &SharedOptions::configPath,
                "default.cfg");
        parser->addValueArg("", "cfg", &SharedOptions::configPath, "", "cfg",
                "config file path (relative to the base config path)", "path");

        parser->addOptionWithDefault<std::string>("", "policy", &SharedOptions::policyPath,
                "pol.pol");
        parser->addValueArg("", "policy", &SharedOptions::policyPath, "", "policy",
                "policy file path (output)", "path");

        parser->addOptionWithDefault<unsigned long>("", "seed", &SharedOptions::seed, 0);
        parser->addValueArg("", "seed", &SharedOptions::seed, "s", "seed",
                "RNG seed; 0=>current time", "ulong");

        parser->addOptionWithDefault<unsigned long>("", "state", &SharedOptions::rngState, 0);
        parser->addValueArg("", "state", &SharedOptions::rngState, "", "state",
                "RNG state", "ulong");

        parser->addOptionWithDefault("", "color", &SharedOptions::hasColorOutput, false);
        parser->addSwitchArg("", "color", &SharedOptions::hasColorOutput, "", "color",
                "use color output", true);

        parser->addOptionWithDefault("", "verbose", &SharedOptions::hasVerboseOutput, false);
        parser->addSwitchArg("", "verbose", &SharedOptions::hasVerboseOutput, "v", "verbose",
                        "use verbose output", true);
    }

    /** Adds simulation-related options for this SharedOptions instance to the given parser;
     * some of these options will be mandatory if simulating, or optional if not simulating.
     */
    static void addSimulationOptions(options::OptionParser *parser, bool simulating) {
        parser->addOptionWithDefault<std::string>("", "log", &SharedOptions::logPath, "log.log");

        parser->addOptionWithDefault("changes", "hasChanges", &SharedOptions::hasChanges, false);
        parser->addOptionWithDefault("changes", "areDynamic", &SharedOptions::areDynamic, false);
        parser->addOptionWithDefault<std::string>("changes", "changesPath",
                &SharedOptions::changesPath, "");

        if (!simulating) {
            parser->addOptionWithDefault<long>("simulation", "nSteps", &SharedOptions::nSimulationSteps, 200);
            parser->addOptionWithDefault<unsigned long>("ABT", "minParticleCount",
                    &Options::minParticleCount, 5000);
            parser->addOptionWithDefault<bool>("ABT", "pruneEveryStep",
                                &Options::pruneEveryStep, false);
            parser->addOptionWithDefault<bool>("ABT", "resetOnChanges",
                                            &Options::resetOnChanges, false);
        } else {
            parser->addOption<long>("simulation", "nSteps", &SharedOptions::nSimulationSteps);
            parser->addOption<unsigned long>("ABT", "minParticleCount", &Options::minParticleCount);
            parser->addOption<bool>("ABT", "pruneEveryStep", &Options::pruneEveryStep);
            parser->addOption<bool>("ABT", "resetOnChanges", &Options::resetOnChanges);
        }

        parser->addOptionWithDefault<long>("simulation", "nRuns", &SharedOptions::nRuns, 1);
        parser->addOptionWithDefault<bool>("simulation", "loadInitialPolicy", &SharedOptions::loadInitialPolicy, false);
        parser->addOptionWithDefault<bool>("simulation", "savePolicy", &SharedOptions::savePolicy, false);
        parser->addOptionWithDefault<bool>("simulation", "saveParticles", &SharedOptions::saveParticles, false);

        if (simulating) {
            // These command line options are for simulation only.
            parser->addValueArg("", "log", &SharedOptions::logPath, "", "log",
                    "file to log changes to", "path");

            parser->addSwitchArg("changes", "hasChanges", &SharedOptions::hasChanges, "",
                    "do-changes", "Set the POMDP model to load changes at runtime", true);

            parser->addSwitchArg("changes", "areDynamic", &SharedOptions::areDynamic, "",
                    "dynamic", "Sets the changes to be dynamic, i.e. apply only to the future, not"
                    " to the past or to alternate futures).", true);

            parser->addValueArg("changes", "changesPath", &SharedOptions::changesPath,
                    "", "changes",
                    "path to file with runtime changes (relative to the base config path)", "path");
            parser->addValueArg<long>("simulation", "nSteps", &SharedOptions::nSimulationSteps,
                    "n", "steps", "Maximum number of steps to simulate", "int");
            parser->addValueArg<unsigned long>("ABT", "minParticleCount",
                    &Options::minParticleCount, "", "min-particles", "Minimum allowable particles"
                            " per belief during simulation - if the count drops below this value,"
                            " extra particles will be resampled via a particle filter.", "int");
            parser->addSwitchArg("ABT", "pruneEveryStep",
                    &Options::pruneEveryStep, "", "prune", "Prune after every step"
                            " of the simulation.", true);
            parser->addSwitchArg("ABT", "resetOnChanges",
                    &Options::resetOnChanges, "", "reset", "Rebuild the tree from"
                            " scratch whenever there are changes to the model", true);
            parser->addSwitchArg("ABT", "pruneEveryStep",
                    &Options::pruneEveryStep, "", "no-prune", "Don't prune after every step"
                            " of the simulation.", false);


            parser->addValueArg<long>("simulation", "nRuns", &SharedOptions::nRuns,
                                "r", "runs", "Number of runs", "int");

            /**parser->addSwitchArg("simulation", "savePolicy", &SharedOptions::savePolicy, "",
                    "save", "save policies to a file after simulation (these files can be very"
                            " large).", true);*/

            parser->addSwitchArg("simulation", "saveParticles", &SharedOptions::saveParticles, "",
                    "save", "save particles to a file after simulation (these files can be very"
                            " large).", true);

            parser->addSwitchArg("simulation", "loadInitialPolicy", &SharedOptions::loadInitialPolicy, "",
                    "no-load", "don't load initial policy---start with empty policy and solve 100% online",
					false);
        }
    }

    /** Adds core ABT options for this SharedOptions instance to the given parser. */
    static void addABTOptions(options::OptionParser *parser) {
        parser->addOption<unsigned long>("ABT", "historiesPerStep", &Options::historiesPerStep);
        parser->addValueArg<unsigned long>("ABT", "historiesPerStep", &Options::historiesPerStep,
                "i", "histories-per-step",
                "number of episodes to sample for each step; 0=>wait for timeout", "int");

        parser->addOptionWithDefault<double>("ABT", "stepTimeout", &Options::stepTimeout, 2.0);
        parser->addValueArg<double>("ABT", "stepTimeout", &Options::stepTimeout,
                "t", "timeout", "step timeout in milliseconds; 0=>no timeout", "real");

        parser->addOption<long>("ABT", "maximumDepth", &Options::maximumDepth);
        parser->addOption<bool>("ABT", "isAbsoluteHorizon", &Options::isAbsoluteHorizon);

        parser->addOption<std::string>("ABT", "searchHeuristic", &SharedOptions::searchHeuristic);
        parser->addOption<std::string>("ABT", "searchStrategy", &SharedOptions::searchStrategy);
        parser->addOptionWithDefault<std::string>("ABT", "recommendationStrategy", &SharedOptions::recommendationStrategy, "max");
        parser->addOption<std::string>("ABT", "estimator", &SharedOptions::estimator);
        parser->addOptionWithDefault<double>("ABT", "maxObservationDistance",
                &SharedOptions::maxObservationDistance, 0.0);
    }

    /** Adds the discountFactor option to the given parser. */
    static void addProblemOptions(options::OptionParser *parser) {
        parser->addOption<double>("problem", "discountFactor", &Options::discountFactor);
        parser->addValueArg<double>("problem", "discountFactor", &Options::discountFactor,
                "d", "discount-factor", "the POMDP discount factor", "real");
    }
};
} /* namespace shared */

#endif /* SHAREDOPTIONS_HPP_ */
