// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

namespace AgOpenWeb.Services.Calibration;

/// <summary>The guided steps of the antenna calibration wizard, in driving order.</summary>
public enum CalibrationWizardStage
{
    /// <summary>No field open — the wizard offers to create a temporary one.</summary>
    Field = 0,

    /// <summary>No guidance line — the wizard offers to create an AB line here.</summary>
    Track = 1,

    /// <summary>Drive the line one way (needs <see cref="CalibrationWizard.MinSamples"/> usable samples).</summary>
    ForwardPass = 2,

    /// <summary>Turn around and drive the same wheel track back.</summary>
    ReversePass = 3,

    /// <summary>Both directions counted; a part of the run must tilt (roll spread gate).</summary>
    RollRange = 4,

    /// <summary>Everything recorded — show the proposal and let the operator apply it.</summary>
    Result = 5,
}

/// <summary>
/// The calibration wizard's step logic, as a pure function of what the app already knows.
/// Order of work: field → line → one way → back → roll spread → result. Each earlier step is a
/// prerequisite, so the panel only ever shows one thing to do at a time; the operator never has
/// to open a field or a track by hand.
///
/// The thresholds come from <see cref="AntennaCalibrationOptions.Default"/>, so the wizard and
/// the fit's own acceptance gates cannot drift apart.
/// </summary>
public static class CalibrationWizard
{
    /// <summary>Usable samples required per direction (the fit's gate).</summary>
    public const int MinSamples = 100;

    /// <summary>Peak-to-peak roll the recording must cover, degrees (the fit's gate).</summary>
    public const double MinRollRangeDegrees = 3.0;

    /// <summary>
    /// Which step the operator is on, from the live state: whether a field and a guidance line
    /// exist, and how many usable samples each direction has.
    /// </summary>
    public static CalibrationWizardStage Evaluate(
        bool hasField,
        bool hasTrack,
        int forwardSamples,
        int reverseSamples,
        double rollRangeDegrees)
    {
        if (!hasField) return CalibrationWizardStage.Field;
        if (!hasTrack) return CalibrationWizardStage.Track;
        if (forwardSamples < MinSamples) return CalibrationWizardStage.ForwardPass;
        if (reverseSamples < MinSamples) return CalibrationWizardStage.ReversePass;
        if (rollRangeDegrees < MinRollRangeDegrees) return CalibrationWizardStage.RollRange;
        return CalibrationWizardStage.Result;
    }

    /// <summary>1-based step number for the panel's "Step n of 6".</summary>
    public static int StepNumber(CalibrationWizardStage stage) => (int)stage + 1;

    /// <summary>Total wizard steps.</summary>
    public static int StepCount => 6;

    /// <summary>
    /// Stable English key for the step, so the client owns the wording (and its translation):
    /// "field", "track", "forward", "reverse", "roll", "result".
    /// </summary>
    public static string Key(CalibrationWizardStage stage) => stage switch
    {
        CalibrationWizardStage.Field => "field",
        CalibrationWizardStage.Track => "track",
        CalibrationWizardStage.ForwardPass => "forward",
        CalibrationWizardStage.ReversePass => "reverse",
        CalibrationWizardStage.RollRange => "roll",
        _ => "result",
    };
}