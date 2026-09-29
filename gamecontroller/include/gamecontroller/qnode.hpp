/**
 * @file /include/gamecontroller/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef gamecontroller_QNODE_HPP_
#define gamecontroller_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#endif
#include <QThread>
#include <string>

#include "gamecontroller/msg/robocupcontroller.hpp"
#include "gamecontroller/msg/gamecontroldata.hpp"
#include "gamecontroller/msg/gamecontrolreturndata.hpp"

/*****************************************************************************
** Class
*****************************************************************************/


class QNode : public QThread
{
  Q_OBJECT
public:
  QNode(int argc, char** argv);
  ~QNode();

  void run();

  gamecontroller::msg::Gamecontroldata gameControlData;
  gamecontroller::msg::Gamecontrolreturndata gameControlReturnData;

  rclcpp::Publisher<gamecontroller::msg::Gamecontroldata>::SharedPtr gamecontrollerPub;

  // config.yaml 파라미터
  std::string pubTopic;
  int teamRobit;
  int teamRobitRed;
  int teamRobitBlue;
  int dataPort;
  int returnPort;

private:
  std::shared_ptr<rclcpp::Node> node;

Q_SIGNALS:
  void rosShutDown();
};

#endif /* gamecontroller_QNODE_HPP_ */
