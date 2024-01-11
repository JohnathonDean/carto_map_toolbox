#ifndef SLAM_TOOLBOX_PANEL_H
#define SLAM_TOOLBOX_PANEL_H

// ROS
#include <ros/ros.h>
#include <rviz/panel.h>
// STL
#include <stdlib.h>
#include <stdio.h>
// QT
#include <QPushButton>
#include <QCheckBox>
#include <QLineEdit>
#include <QComboBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QtGui>
#include <QLabel>
#include <QFrame>
#include <QRadioButton>

#include <thread>

// msgs
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "carto_map/SaveMap.h"
#include "carto_map/OptimizeSubmap.h"
#include "carto_map/OptimizeSubmapPose.h"
#include "carto_map/GetSubmapPose.h"
#include "carto_map/RemoveSubmap.h"
#include "carto_map/RemoveTrajectory.h"
#include "carto_map/SetInteractiveMode.h"

class QLineEdit;
class QSpinBox;
class QComboBox;

#include <rviz/panel.h>

namespace slam_toolbox
{

class CartoSlamToolboxPlugin : public rviz::Panel
{
  Q_OBJECT

public:
  CartoSlamToolboxPlugin(QWidget* parent = 0);
  ~CartoSlamToolboxPlugin();

public Q_SLOTS:
protected Q_SLOTS:
  void SaveMap();
  void RemoveSubmap();
  void RemoveTrajectory();
  void GetSubmapPose();
  void UpdateSubmapPose();
  void OptimizeSubmapPose();
  void MatchOptimizeSubmapPose();
  void ComputeOverlappedSubmaps();
  void InteractiveCb();
  void ClearChanges();

protected:
  QVBoxLayout* _vbox;
  QHBoxLayout* _hbox1;
  QHBoxLayout* _hbox2;
  QHBoxLayout* _hbox3;
  QHBoxLayout* _hbox4;
  QHBoxLayout* _hbox5;
  QHBoxLayout* _hbox6;
  QHBoxLayout* _hbox7;
  QHBoxLayout* _hbox8;
  QHBoxLayout* _hbox9;
  QHBoxLayout* _hbox10;

  QPushButton* _button1;
  QPushButton* _button2;
  QPushButton* _button3;
  QPushButton* _button4;
  QPushButton* _button5;
  QPushButton* _button6;
  QPushButton* _button7;
  QPushButton* _button8;
  QPushButton* _button9;
  QPushButton* _button10;

  QLineEdit* _line1;
  QLineEdit* _line2;
  QLineEdit* _line3;
  QLineEdit* _line4;
  QLineEdit* _line5;
  QLineEdit* _line6;
  QLineEdit* _line7;
  QLineEdit* _line8;
  QLineEdit* _line9;
  QLineEdit* _line10;

  QCheckBox* _check1;
  QCheckBox* _check2;

  QRadioButton* _radio1;
  QRadioButton* _radio2;
  QRadioButton* _radio3;
  QRadioButton* _radio4;

  QLabel* _label1;
  QLabel* _label2;
  QLabel* _label3;
  QLabel* _label4;
  QLabel* _label5;
  QLabel* _label6;
  QLabel* _label7;
  QLabel* _label8;

  QFrame* _line;

  ros::ServiceClient save_map_;
  ros::ServiceClient optimize_submap_;
  ros::ServiceClient optimize_submap_pose_;
  ros::ServiceClient remove_submap_;
  ros::ServiceClient remove_trajectory_;
  ros::ServiceClient compute_overlap_submap_;
  ros::ServiceClient get_submap_pose_;
  ros::ServiceClient set_interactive_mode_;
  ros::ServiceClient clear_change_;

  std::thread* _thread;
  bool paused_measure = false, interactive = false;

};

} // end namespace

#endif
