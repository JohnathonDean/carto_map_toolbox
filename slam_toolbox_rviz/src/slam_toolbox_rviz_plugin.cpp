// Header
#include "slam_toolbox_rviz_plugin.h"
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

// ROS
#include <tf/tf.h>
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(slam_toolbox::CartoSlamToolboxPlugin, rviz::Panel)

namespace slam_toolbox
{

CartoSlamToolboxPlugin::CartoSlamToolboxPlugin(QWidget* parent):
    rviz::Panel(parent),
    _thread(NULL)
{
  ros::NodeHandle nh;
  interactive = false;

  save_map_ = nh.serviceClient<carto_map::SaveMap>("/carto_map/save_map");
  optimize_submap_ = nh.serviceClient<carto_map::OptimizeSubmap>("/carto_map/optimize_submap");
  optimize_submap_pose_ = nh.serviceClient<carto_map::OptimizeSubmapPose>("/carto_map/optimize_pose_submap");
  get_submap_pose_ = nh.serviceClient<carto_map::GetSubmapPose>("/carto_map/get_submap_pose");
  remove_submap_ = nh.serviceClient<carto_map::RemoveSubmap>("/carto_map/remove_submap");
  remove_trajectory_ = nh.serviceClient<carto_map::RemoveTrajectory>("/carto_map/remove_trajectory");
  compute_overlap_submap_ = nh.serviceClient<std_srvs::Trigger>("/carto_map/compute_overlap_submap");
  set_interactive_mode_ = nh.serviceClient<carto_map::SetInteractiveMode>("/carto_map/set_interactive_mode");
  clear_change_ = nh.serviceClient<std_srvs::Trigger>("/carto_map/clear_move_nodes");

  _vbox = new QVBoxLayout();
  _hbox1 = new QHBoxLayout();
  _hbox2 = new QHBoxLayout();
  _hbox3 = new QHBoxLayout();
  _hbox4 = new QHBoxLayout();
  _hbox5 = new QHBoxLayout();
  _hbox6 = new QHBoxLayout();
  _hbox7 = new QHBoxLayout();
  _hbox8 = new QHBoxLayout();

  QFrame* _line = new QFrame();
  _line->setFrameShape(QFrame::HLine);
  _line->setFrameShadow(QFrame::Sunken);

  _button1 = new QPushButton(this);
  _button1->setText("Remove Submap");
  connect(_button1, SIGNAL(clicked()), this, SLOT(RemoveSubmap()));
  _button2 = new QPushButton(this);
  _button2->setText("Remove Trajectory");
  connect(_button2, SIGNAL(clicked()), this, SLOT(RemoveTrajectory()));
  _button3 = new QPushButton(this);
  _button3->setText("Save Map");
  connect(_button3, SIGNAL(clicked()), this, SLOT(SaveMap()));
  _button4 = new QPushButton(this);
  _button4->setText("Get Submap");
  connect(_button4, SIGNAL(clicked()), this, SLOT(GetSubmapPose()));
  _button5 = new QPushButton(this);
  _button5->setText("Update Pose");
  connect(_button5, SIGNAL(clicked()), this, SLOT(UpdateSubmapPose()));
  _button6 = new QPushButton(this);
  _button6->setText("Optimize Pose");
  connect(_button6, SIGNAL(clicked()), this, SLOT(OptimizeSubmapPose()));
  _button7 = new QPushButton(this);
  _button7->setText("Align To Submap");
  connect(_button7, SIGNAL(clicked()), this, SLOT(MatchOptimizeSubmapPose()));
  _button8 = new QPushButton(this);
  _button8->setText("Overlapping Compute");
  connect(_button8, SIGNAL(clicked()), this, SLOT(ComputeOverlappedSubmaps()));
  _button9 = new QPushButton(this);
  _button9->setText(" Interactive OFF");
  connect(_button9, SIGNAL(clicked()), this, SLOT(InteractiveCb()));
  _button10 = new QPushButton(this);
  _button10->setText("Clear Change");
  connect(_button10, SIGNAL(clicked()), this, SLOT(ClearChanges()));

  _label2 = new QLabel(this);
  _label2->setText("Optimize Submap Pose");

  _label6 = new QLabel(this);
  _label6->setText("X");
  _label6->setAlignment(Qt::AlignCenter);
  _label7 = new QLabel(this);
  _label7->setText("Y");
  _label7->setAlignment(Qt::AlignCenter);
  _label8 = new QLabel(this);
  _label8->setText("θ");
  _label8->setAlignment(Qt::AlignCenter);

  _line1 = new QLineEdit();
  _line2 = new QLineEdit();
  _line3 = new QLineEdit();
  _line4 = new QLineEdit();
  _line5 = new QLineEdit();
  _line6 = new QLineEdit();
  _line7 = new QLineEdit();
  _line8 = new QLineEdit();
  _line9 = new QLineEdit();
  _line10 = new QLineEdit();

  _button1->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _button2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _button3->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _button4->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _button5->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _button6->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _button7->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _button8->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _button9->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _button10->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  _line1->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _line2->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _line3->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _line4->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _line5->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _line6->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _line7->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _line8->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _line9->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  _line10->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);


  _hbox1->addWidget(_button1);
  _hbox1->addWidget(_line1);
  _hbox1->addWidget(_line2);

  _hbox2->addWidget(_button2);
  _hbox2->addWidget(_line3);

  _hbox3->addWidget(_button8);
  _hbox3->addWidget(_button3);

  _hbox4->addWidget(_button4);
  _hbox4->addWidget(_line4);
  _hbox4->addWidget(_line5);

  _hbox5->addWidget(_label6);
  _hbox5->addWidget(_line6);
  _hbox5->addWidget(_label7);
  _hbox5->addWidget(_line7);
  _hbox5->addWidget(_label8);
  _hbox5->addWidget(_line8);

  _hbox6->addWidget(_button5);
  _hbox6->addWidget(_button6);

  _hbox7->addWidget(_button7);
  _hbox7->addWidget(_line9);
  _hbox7->addWidget(_line10);

  _hbox8->addWidget(_button9);
  _hbox8->addWidget(_button10);

  _vbox->addLayout(_hbox1);
  _vbox->addLayout(_hbox2);
  _vbox->addLayout(_hbox3);
  _vbox->addWidget(_line);
  _vbox->addWidget(_label2);
  _vbox->addLayout(_hbox4);
  _vbox->addLayout(_hbox5);
  _vbox->addLayout(_hbox6);
  _vbox->addLayout(_hbox7);
  _vbox->addLayout(_hbox8);

  setLayout(_vbox);

  // _thread = new std::thread(&CartoSlamToolboxPlugin::updateCheckStateIfExternalChange, this);
}

CartoSlamToolboxPlugin::~CartoSlamToolboxPlugin()
{
  if (_thread)
  {
    delete _thread;
  }
}

void CartoSlamToolboxPlugin::SaveMap()
{
  carto_map::SaveMap msg;
  msg.request.filename = "###";
  save_map_.call(msg);
}

void CartoSlamToolboxPlugin::RemoveSubmap() {
  carto_map::RemoveSubmap msg;
  msg.request.trajectory_id = _line1->text().toInt();
  msg.request.submap_index = _line2->text().toInt();
  remove_submap_.call(msg);
}

void CartoSlamToolboxPlugin::RemoveTrajectory() {
  carto_map::RemoveTrajectory msg;
  msg.request.trajectory_id = _line3->text().toInt();
  remove_trajectory_.call(msg);
}

void CartoSlamToolboxPlugin::GetSubmapPose() {
  carto_map::GetSubmapPose msg;
  msg.request.trajectory_id = _line4->text().toInt();
  msg.request.submap_index = _line5->text().toInt();
  ROS_INFO("GetSubmapPose.");
  if(get_submap_pose_.call(msg)){
    _line6->setText(QString::number(msg.response.x));
    _line7->setText(QString::number(msg.response.y));
    _line8->setText(QString::number(msg.response.theta));
  }
}

void CartoSlamToolboxPlugin::UpdateSubmapPose() {
  carto_map::OptimizeSubmapPose msg;
  msg.request.trajectory_id = _line4->text().toInt();
  msg.request.submap_index = _line5->text().toInt();
  msg.request.x = _line6->text().toDouble();
  msg.request.y = _line7->text().toDouble();
  msg.request.theta = _line8->text().toDouble();
  msg.request.optimize = 0;
  optimize_submap_pose_.call(msg);
}

void CartoSlamToolboxPlugin::OptimizeSubmapPose() {
  carto_map::OptimizeSubmapPose msg;
  msg.request.trajectory_id = _line4->text().toInt();
  msg.request.submap_index = _line5->text().toInt();
  msg.request.x = _line6->text().toDouble();
  msg.request.y = _line7->text().toDouble();
  msg.request.theta = _line8->text().toDouble();
  msg.request.optimize = 1;
  optimize_submap_pose_.call(msg);
}

void CartoSlamToolboxPlugin::MatchOptimizeSubmapPose() {
  carto_map::OptimizeSubmap msg;
  msg.request.trajectory_id = _line4->text().toInt();
  msg.request.submap_index = _line5->text().toInt();
  msg.request.trajectory_id_source = _line9->text().toInt();
  msg.request.submap_index_source = _line10->text().toInt();
  optimize_submap_.call(msg);
}

void CartoSlamToolboxPlugin::ComputeOverlappedSubmaps() {
  std_srvs::Trigger msg;
  compute_overlap_submap_.call(msg);
}

void CartoSlamToolboxPlugin::InteractiveCb() {
  carto_map::SetInteractiveMode msg;
  if (interactive) {
    msg.request.cmd = 0;
    interactive = false;
    _button9->setText(" Interactive OFF");
  }else{
    msg.request.cmd = 1;
    interactive = true;
    _button9->setText(" Interactive ON");
  }
  set_interactive_mode_.call(msg);
}

void CartoSlamToolboxPlugin::ClearChanges() {
  std_srvs::Trigger msg;
  clear_change_.call(msg);
}

} // end namespace
