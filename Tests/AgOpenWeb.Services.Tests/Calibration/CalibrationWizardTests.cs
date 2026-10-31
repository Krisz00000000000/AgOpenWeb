// AgOpenWeb
// Copyright (C) 2024-2026 AgOpenWeb Contributors
// SPDX-License-Identifier: Apache-2.0

using AgOpenWeb.Services.Calibration;
using NUnit.Framework;

namespace AgOpenWeb.Services.Tests.Calibration;

/// <summary>
/// The wizard's step order: field → line → one way → back → roll spread → result. Each step is
/// a prerequisite of the next, so the panel only ever shows one thing to do — and nobody has to
/// open a field or a track by hand.
/// </summary>
[TestFixture]
public class CalibrationWizardTests
{
    [Test]
    public void NoField_AsksForAFieldFirst()
    {
        Assert.That(CalibrationWizard.Evaluate(hasField: false, hasTrack: false, 0, 0, 0),
            Is.EqualTo(CalibrationWizardStage.Field));
    }

    [Test]
    public void FieldButNoLine_AsksForTheAbLine()
    {
        Assert.That(CalibrationWizard.Evaluate(hasField: true, hasTrack: false, 0, 0, 0),
            Is.EqualTo(CalibrationWizardStage.Track));
    }

    [Test]
    public void BothReady_DrivesOneWayFirst()
    {
        Assert.That(CalibrationWizard.Evaluate(true, true, 0, 0, 0),
            Is.EqualTo(CalibrationWizardStage.ForwardPass));
    }

    [Test]
    public void OneWayShort_StaysOnTheForwardPass()
    {
        Assert.That(CalibrationWizard.Evaluate(true, true, CalibrationWizard.MinSamples - 1, 0, 0),
            Is.EqualTo(CalibrationWizardStage.ForwardPass));
    }

    [Test]
    public void ForwardDone_AsksForTheWayBack()
    {
        Assert.That(CalibrationWizard.Evaluate(true, true, CalibrationWizard.MinSamples, 0, 0),
            Is.EqualTo(CalibrationWizardStage.ReversePass));
    }

    [Test]
    public void BothDirectionsDone_AsksForRollSpread()
    {
        Assert.That(CalibrationWizard.Evaluate(true, true, CalibrationWizard.MinSamples,
                CalibrationWizard.MinSamples, CalibrationWizard.MinRollRangeDegrees - 0.1),
            Is.EqualTo(CalibrationWizardStage.RollRange));
    }

    [Test]
    public void EveryGateMet_ShowsTheResult()
    {
        Assert.That(CalibrationWizard.Evaluate(true, true, CalibrationWizard.MinSamples,
                CalibrationWizard.MinSamples, CalibrationWizard.MinRollRangeDegrees),
            Is.EqualTo(CalibrationWizardStage.Result));
    }

    [Test]
    public void StepsAreNumberedFromOne_AndTheKeysAreStable()
    {
        Assert.That(CalibrationWizard.StepCount, Is.EqualTo(6));
        Assert.That(CalibrationWizard.StepNumber(CalibrationWizardStage.Field), Is.EqualTo(1));
        Assert.That(CalibrationWizard.StepNumber(CalibrationWizardStage.Result), Is.EqualTo(6));

        // The client translates by these keys, so they must not change silently.
        Assert.That(CalibrationWizard.Key(CalibrationWizardStage.Field), Is.EqualTo("field"));
        Assert.That(CalibrationWizard.Key(CalibrationWizardStage.Track), Is.EqualTo("track"));
        Assert.That(CalibrationWizard.Key(CalibrationWizardStage.ForwardPass), Is.EqualTo("forward"));
        Assert.That(CalibrationWizard.Key(CalibrationWizardStage.ReversePass), Is.EqualTo("reverse"));
        Assert.That(CalibrationWizard.Key(CalibrationWizardStage.RollRange), Is.EqualTo("roll"));
        Assert.That(CalibrationWizard.Key(CalibrationWizardStage.Result), Is.EqualTo("result"));
    }

    [Test]
    public void WizardGatesMatchTheFitsOwnGates()
    {
        // One source of truth: the wizard must not promise a shorter run than the fit accepts.
        Assert.That(CalibrationWizard.MinSamples, Is.EqualTo(AntennaCalibrationOptions.Default.MinSamplesPerDirection));
        Assert.That(CalibrationWizard.MinRollRangeDegrees, Is.EqualTo(AntennaCalibrationOptions.Default.MinRollRangeDegrees));
    }
}