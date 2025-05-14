function TestFrameTransformations()
  pos_lla_truth = [7.952458*pi/180; 14.401053*pi/180; 191.12]; 
  pos_lla_ref = pos_lla_truth;
  pos_ecef_truth = [6118892.40062107; 1571184.4112357; 876599.276536116];

  pos2_ned_truth = [1.2, 3.8, -10.2];
  pos2_ecef_truth = [6118901.07921937, 1571190.56296585, 876601.876179878];

  pos3_lla_truth = [0; 0; 0];
  pos3_ecef_truth = [6378137; 0; 0];

  pos4_lla_truth = [pi/2; 0; 0];
  pos4_ecef_truth = [0; 0; 6356752.31424518];

  pos5_lla_truth = [-pi/2; 0; 0];
  pos5_ecef_truth = [0; 0; -6356752.31424518];


  Test_From_To(pos_lla_truth, pos_ecef_truth, pos_lla_ref, 1e-5, "LLA",  "ECEF");
  Test_From_To(pos_ecef_truth, pos_lla_truth, pos_lla_ref, 1e-5, "ECEF", "LLA");
  Test_From_To(pos3_lla_truth, pos3_ecef_truth, pos_lla_ref, 1e-5, "LLA",  "ECEF");
  Test_From_To(pos3_ecef_truth, pos3_lla_truth, pos_lla_ref, 1e-5, "ECEF", "LLA");
  Test_From_To(pos4_lla_truth, pos4_ecef_truth, pos_lla_ref, 1e-5, "LLA",  "ECEF");
  Test_From_To(pos4_ecef_truth, pos4_lla_truth, pos_lla_ref, 1e-5, "ECEF", "LLA");
  Test_From_To(pos5_lla_truth, pos5_ecef_truth, pos_lla_ref, 1e-5, "LLA",  "ECEF");
  Test_From_To(pos5_ecef_truth, pos5_lla_truth, pos_lla_ref, 1e-5, "ECEF", "LLA");

  Test_From_To(pos2_ned_truth, pos2_ecef_truth, pos_lla_ref, 1e-5, "NED", "ECEF");
  Test_From_To(pos2_ecef_truth, pos2_ned_truth, pos_lla_ref, 1e-5, "ECEF", "NED");
end

function Test_From_To(pos_from, pos_truth, lla_ref, tol, fromFrame, toFrame)
  pos_test = ChangeFrame(pos_from, lla_ref, fromFrame, toFrame);
  error = norm(pos_test - pos_truth(:));
  if error > tol
    fprintf("%s to %s test FAILED: Error = %f\n", fromFrame, toFrame, error)
    fprintf("\tTest: [%f, %f, %f]\n", pos_test(1), pos_test(2), pos_test(3))
    fprintf("\tTruth: [%f, %f, %f]\n", pos_truth(1), pos_truth(2), pos_truth(3))
  else
    fprintf("%s to %s test PASSED\n", fromFrame, toFrame)
  end
end

