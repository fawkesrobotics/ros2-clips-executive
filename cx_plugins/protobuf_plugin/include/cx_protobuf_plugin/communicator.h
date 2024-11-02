
/***************************************************************************
 *  communicator.h - protobuf network communication for CLIPS
 *
 *  Created: Tue Apr 16 13:41:13 2013
 *  Copyright  2013-2014  Tim Niemueller [www.niemueller.de]
 *             2021       Till Hofmann <hofmann@kbsg.rwth-aachen.de>
 ****************************************************************************/

/*  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * - Neither the name of the authors nor the names of its contributors
 *   may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _PROTOBUF_CLIPS_COMMUNICATOR_H_
#define _PROTOBUF_CLIPS_COMMUNICATOR_H_

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include <protobuf_comm/server.h>

#include <clips_ns/clips.h>
#include <list>
#include <map>
#include <mutex>

namespace protobuf_comm {
class ProtobufStreamClient;
class ProtobufBroadcastPeer;
} // namespace protobuf_comm

namespace fawkes {
class Logger;
}

namespace protobuf_clips {

class ClipsProtobufCommunicator {
public:
  ClipsProtobufCommunicator(clips::Environment *env, std::mutex &env_mutex,
                            rclcpp_lifecycle::LifecycleNode::WeakPtr parent);
  ClipsProtobufCommunicator(clips::Environment *env, std::mutex &env_mutex,
                            std::vector<std::string> &proto_path,
                            rclcpp_lifecycle::LifecycleNode::WeakPtr parent);
  ~ClipsProtobufCommunicator();

  void enable_server(int port);
  void disable_server();

  /** Signal invoked for a message that has been sent to a server client.
   * @return signal
   */
  boost::signals2::signal<void(protobuf_comm::ProtobufStreamServer::ClientID,
                               std::shared_ptr<google::protobuf::Message>)> &
  signal_server_sent() {
    return sig_server_sent_;
  }

  /** Signal invoked for a message that has been sent to a client.
   * @return signal
   */
  boost::signals2::signal<void(std::string, unsigned short,
                               std::shared_ptr<google::protobuf::Message>)> &
  signal_client_sent() {
    return sig_client_sent_;
  }

  /** Signal invoked for a message that has been sent via broadcast.
   * @return signal
   */
  boost::signals2::signal<void(long,
                               std::shared_ptr<google::protobuf::Message>)> &
  signal_peer_sent() {
    return sig_peer_sent_;
  }

private:
  void setup_clips();

  clips::UDFValue clips_pb_register_type(std::string full_name);
  clips::UDFValue clips_pb_field_names(void *msgptr);
  clips::UDFValue clips_pb_has_field(void *msgptr, std::string field_name);
  clips::UDFValue clips_pb_field_value(void *msgptr, std::string field_name);
  clips::UDFValue clips_pb_field_type(void *msgptr, std::string field_name);
  clips::UDFValue clips_pb_field_label(void *msgptr, std::string field_name);
  clips::UDFValue clips_pb_field_list(void *msgptr, std::string field_name);
  clips::UDFValue clips_pb_field_is_list(void *msgptr, std::string field_name);
  clips::UDFValue clips_pb_create(std::string full_name);
  clips::UDFValue clips_pb_ref(void *msgptr);
  void clips_pb_destroy(void *msgptr);
  void clips_pb_set_field(void *msgptr, std::string field_name,
                          clips::UDFValue value);
  void clips_pb_add_list(void *msgptr, std::string field_name,
                         clips::UDFValue value);
  void clips_pb_send(long int client_id, void *msgptr);
  std::string clips_pb_tostring(void *msgptr);
  long int clips_pb_client_connect(std::string host, int port);
  void clips_pb_disconnect(long int client_id);
  void clips_pb_broadcast(long int peer_id, void *msgptr);

  long int clips_pb_peer_create(std::string host, int port);
  long int clips_pb_peer_create_local(std::string host, int send_port,
                                      int recv_port);
  long int clips_pb_peer_create_crypto(std::string host, int port,
                                       std::string crypto_key = "",
                                       std::string cipher = "");
  long int clips_pb_peer_create_local_crypto(std::string host, int send_port,
                                             int recv_port,
                                             std::string crypto_key = "",
                                             std::string cipher = "");
  void clips_pb_peer_destroy(long int peer_id);
  void clips_pb_peer_setup_crypto(long int peer_id, std::string crypto_key,
                                  std::string cipher);

  typedef enum { CT_SERVER, CT_CLIENT, CT_PEER } ClientType;
  void clips_assert_message(std::pair<std::string, unsigned short> &endpoint,
                            uint16_t comp_id, uint16_t msg_type,
                            std::shared_ptr<google::protobuf::Message> &msg,
                            ClientType ct, long int client_id = 0);
  void handle_server_client_connected(
      protobuf_comm::ProtobufStreamServer::ClientID client,
      boost::asio::ip::tcp::endpoint &endpoint);
  void handle_server_client_disconnected(
      protobuf_comm::ProtobufStreamServer::ClientID client,
      const boost::system::error_code &error);

  void
  handle_server_client_msg(protobuf_comm::ProtobufStreamServer::ClientID client,
                           uint16_t component_id, uint16_t msg_type,
                           std::shared_ptr<google::protobuf::Message> msg);

  void handle_server_client_fail(
      protobuf_comm::ProtobufStreamServer::ClientID client,
      uint16_t component_id, uint16_t msg_type, std::string msg);

  void handle_peer_msg(long int peer_id,
                       boost::asio::ip::udp::endpoint &endpoint,
                       uint16_t component_id, uint16_t msg_type,
                       std::shared_ptr<google::protobuf::Message> msg);
  void handle_peer_recv_error(long int peer_id,
                              boost::asio::ip::udp::endpoint &endpoint,
                              std::string msg);
  void handle_peer_send_error(long int peer_id, std::string msg);

  void handle_client_connected(long int client_id);
  void handle_client_disconnected(long int client_id,
                                  const boost::system::error_code &error);
  void handle_client_msg(long int client_id, uint16_t comp_id,
                         uint16_t msg_type,
                         std::shared_ptr<google::protobuf::Message> msg);
  void handle_client_receive_fail(long int client_id, uint16_t comp_id,
                                  uint16_t msg_type, std::string msg);

  static std::string to_string(const clips::UDFValue &v);

private:
  clips::Environment *clips_;
  std::mutex &clips_mutex_;

  std::unique_ptr<protobuf_comm::MessageRegister> message_register_;
  std::unique_ptr<protobuf_comm::ProtobufStreamServer> server_;

  boost::signals2::signal<void(protobuf_comm::ProtobufStreamServer::ClientID,
                               std::shared_ptr<google::protobuf::Message>)>
      sig_server_sent_;
  boost::signals2::signal<void(std::string, unsigned short,
                               std::shared_ptr<google::protobuf::Message>)>
      sig_client_sent_;
  boost::signals2::signal<void(long int,
                               std::shared_ptr<google::protobuf::Message>)>
      sig_peer_sent_;

  std::mutex map_mutex_;
  long int next_client_id_;

  std::map<long int, protobuf_comm::ProtobufStreamServer::ClientID>
      server_clients_;
  typedef std::map<protobuf_comm::ProtobufStreamServer::ClientID, long int>
      RevServerClientMap;
  RevServerClientMap rev_server_clients_;
  std::map<long int, std::unique_ptr<protobuf_comm::ProtobufStreamClient>>
      clients_;
  std::map<long int, std::unique_ptr<protobuf_comm::ProtobufBroadcastPeer>>
      peers_;

  std::map<long int, std::pair<std::string, unsigned short>> client_endpoints_;

  std::list<std::string> functions_;
  /// Reference to parent node to get ros time
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_;

  std::unordered_map<void *, std::shared_ptr<google::protobuf::Message>>
      messages_;
};

} // end namespace protobuf_clips

#endif
