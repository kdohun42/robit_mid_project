/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date August 2024
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/ebimu/main_window.hpp"
#include "../include/ebimu/qnode.hpp"

extern rclcpp::Publisher<humanoid_interfaces::msg::ImuMsg>::SharedPtr imu_publisher_;

namespace e2box_imu {

MainWindow::MainWindow(int argc, char** argv, QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);
  uiUpdate(0.0, 0.0, 0.0);

  ui->dial->setRange(-180, 180);
  ui->dial->setWrapping(true);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode(argc, argv);

  QObject::connect(qnode, SIGNAL(dial_update(double,double,double)), this, SLOT(imu_return(double,double,double)));
  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
}

void MainWindow::closeEvent(QCloseEvent* event)
{
  rclcpp::shutdown();
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete ui;
}
void MainWindow::imu_return(double roll, double pitch, double yaw)
{ 
  switch(qnode->imu_flag_)
  {
    case 0:
      on_pushButton_set_clicked();
      break;
    case 90:
      on_pushButton_set_p90_clicked();
      break;
    case -90:
      on_pushButton_set_n90_clicked();
      break;
    case 180:
      on_pushButton_set_180_clicked();
      break;
    default:
      
      break;
  }
  qnode->clearFlag();
  humanoid_interfaces::msg::ImuMsg imu_data;
  imu_data.yaw = yaw;
  imu_data.roll = roll;
  imu_data.pitch = pitch;

  std::cout.precision(4);
  std::cout << std::endl;
  std::cout << " YAW  | " << yaw << std::endl;
  std::cout << " ROLL | " << roll << std::endl;
  std::cout << " PITCH| " << pitch << std::endl;
  std::cout << std::endl;

  imu_publisher_->publish(imu_data);

  uiUpdate(yaw, roll, pitch);
}

void MainWindow::uiUpdate(double yaw, double roll, double pitch)
{
  suppress_slider_sync_ = true;
  ui->dial->setValue(static_cast<int>(-yaw));
  ui->verticalSlider->setValue(static_cast<int>(pitch));
  ui->horizontalSlider->setValue(static_cast<int>(roll));
  suppress_slider_sync_ = false;
}

void MainWindow::on_pushButton_set_clicked()
{
    std::cout<<"set 0"<<std::endl;
    qnode->setYawOffset(0.0);
    qnode->publishSetYaw(0.0);
}

void MainWindow::on_pushButton_2_clicked()
{
    std::cout<<"set roll"<<std::endl;
    qnode->setRollOffset(0.0);
}

void MainWindow::on_pushButton_clicked()
{
    std::cout<<"set pitch"<<std::endl;
    qnode->setPitchOffset(0.0);
}

void MainWindow::on_pushButton_set_p90_clicked()
{
    std::cout<<"set +90"<<std::endl;
    qnode->setYawOffset(90.0);
    qnode->publishSetYaw(90.0);
}

void MainWindow::on_pushButton_set_n90_clicked()
{
    std::cout<<"set -90"<<std::endl;
    qnode->setYawOffset(-90.0);
    qnode->publishSetYaw(-90.0);
}

void MainWindow::on_pushButton_set_180_clicked()
{
    std::cout<<"set 180"<<std::endl;
    qnode->setYawOffset(180.0);
    qnode->publishSetYaw(180.0);
}

void MainWindow::on_verticalSlider_valueChanged(int value)
{
    if (suppress_slider_sync_)
        return;
    qnode->setPitchOffset(static_cast<double>(value));
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    if (suppress_slider_sync_)
        return;
    qnode->setRollOffset(static_cast<double>(value));
}
}
