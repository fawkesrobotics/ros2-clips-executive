
/***************************************************************************
 *  communicator.cpp - protobuf network communication for CLIPS
 *
 *  Created: Tue Apr 16 13:51:14 2013
 *  Copyright  2013  Tim Niemueller [www.niemueller.de]
 *             2021  Till Hofmann <hofmann@kbsg.rwth-aachen.de>
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

#include <cx_protobuf_plugin/communicator.h>
#include <google/protobuf/descriptor.h>
#include <protobuf_comm/client.h>
#include <protobuf_comm/peer.h>
#include <protobuf_comm/server.h>
#include <spdlog/spdlog.h>

#include <boost/format.hpp>
#include <filesystem>
#include <format>
#include <string>

using namespace google::protobuf;
using namespace protobuf_comm;
using namespace boost::placeholders;

namespace protobuf_clips {

/** @class ClipsProtobufCommunicator <protobuf_clips/communicator.h>
 * CLIPS protobuf integration class.
 * This class adds functionality related to protobuf to a given CLIPS
 * environment. It supports the creation of communication channels
 * through protobuf_comm. An instance maintains its own message register
 * shared among server, peer, and clients.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param env CLIPS environment to which to provide the protobuf functionality
 * @param env_mutex mutex to lock when operating on the CLIPS environment.
 */
ClipsProtobufCommunicator::ClipsProtobufCommunicator(clips::Environment *env,
                                                     std::mutex &env_mutex)
    : clips_(env), clips_mutex_(env_mutex), server_(NULL), next_client_id_(0) {
  message_register_ = new MessageRegister();
  setup_clips();
}

/** Constructor.
 * @param env CLIPS environment to which to provide the protobuf functionality
 * @param env_mutex mutex to lock when operating on the CLIPS environment.
 * @param proto_path proto path passed to a newly instantiated message register
 */
ClipsProtobufCommunicator::ClipsProtobufCommunicator(
    clips::Environment *env, std::mutex &env_mutex,
    std::vector<std::string> &proto_path)
    : clips_(env), clips_mutex_(env_mutex), server_(NULL), next_client_id_(0) {
  message_register_ = new MessageRegister(proto_path);
  setup_clips();
}

/** Destructor. */
ClipsProtobufCommunicator::~ClipsProtobufCommunicator() {
  {
    for (auto f : functions_) {
      clips::RemoveUDF(clips_, f.c_str());
    }
    functions_.clear();
  }

  for (auto c : clients_) {
    delete c.second;
  }
  clients_.clear();

  delete message_register_;
  delete server_;
}

/** Setup CLIPS environment. */
void ClipsProtobufCommunicator::setup_clips() {
  // std::lock_guard<std::mutex> lock(clips_mutex_);
  using namespace clips;
  clips::AddUDF(
      clips_, "pb-register-type", "b", 1, 1, ";sy",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue full_name;
        if(!clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &full_name)) {
          SPDLOG_ERROR("pb-register-type: unexpected types, expected addr");
          clips::UDFThrowError(udfc);
          return;
        }
        *out =
            instance->clips_pb_register_type(full_name.lexemeValue->contents);
      },
      "clips_pb_register_type", this);
  functions_.push_back("pb-register-type");

  clips::AddUDF(
      clips_, "pb-field-names", "m", 1, 1, ";e",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue msgptr;
        if(!clips::UDFNthArgument(udfc, 1, clips::EXTERNAL_ADDRESS_BIT, &msgptr)) {
          SPDLOG_ERROR("pb-field-names: unexpected types, expected addr");
          clips::UDFThrowError(udfc);
          return;
        }
        *out = instance->clips_pb_field_names(
            msgptr.externalAddressValue->contents);
      },
      "clips_pb_field_names", this);
  functions_.push_back("pb-field-names");

  clips::AddUDF(
      clips_, "pb-field-type", "y", 2, 2, ";e;sy",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue msgptr, field_name;
        if(!clips::UDFNthArgument(udfc, 1, clips::EXTERNAL_ADDRESS_BIT, &msgptr) ||
           !clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &field_name)) {
          SPDLOG_ERROR("pb-field-type: unexpected types, expected addr;lex");
          clips::UDFThrowError(udfc);
          return;
        }
        *out =
            instance->clips_pb_field_type(msgptr.externalAddressValue->contents,
                                          field_name.lexemeValue->contents);
      },
      "clips_pb_field_type", this);
  functions_.push_back("pb-field-type");

  clips::AddUDF(
      clips_, "pb-has-field", "b", 2, 2, ";e;sy",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue msgptr, field_name;
        if(!clips::UDFNthArgument(udfc, 1, clips::EXTERNAL_ADDRESS_BIT, &msgptr) ||
           !clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &field_name)) {
          SPDLOG_ERROR("pb-has-field: unexpected types, expected addr;lex");
          clips::UDFThrowError(udfc);
          return;
        }
        *out =
            instance->clips_pb_has_field(msgptr.externalAddressValue->contents,
                                         field_name.lexemeValue->contents);
      },
      "clips_pb_has_field", this);
  functions_.push_back("pb-has-field");

  clips::AddUDF(
      clips_, "pb-field-label", "y", 2, 2, ";e;sy",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue msgptr, field_name;
        if(!clips::UDFNthArgument(udfc, 1, clips::EXTERNAL_ADDRESS_BIT, &msgptr) ||
           !clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &field_name)) {
          SPDLOG_ERROR("pb-field-label: unexpected types, expected addr;lex");
          clips::UDFThrowError(udfc);
          return;
        }
        *out = instance->clips_pb_field_label(
            msgptr.externalAddressValue->contents,
            field_name.lexemeValue->contents);
      },
      "clips_pb_field_label", this);
  functions_.push_back("pb-field-label");

  clips::AddUDF(
      clips_, "pb-field-value", "*", 2, 2, ";e;sy",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue msgptr, field_name;
        if(!clips::UDFNthArgument(udfc, 1, clips::EXTERNAL_ADDRESS_BIT, &msgptr) ||
           !clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &field_name)) {
          SPDLOG_ERROR("pb-field-value: unexpected types, expected addr;lex");
          clips::UDFThrowError(udfc);
          return;
        }
        *out = instance->clips_pb_field_value(
            msgptr.externalAddressValue->contents,
            field_name.lexemeValue->contents);
      },
      "clips_pb_field_value", this);
  functions_.push_back("pb-field-value");

  clips::AddUDF(
      clips_, "pb-field-list", "ym", 2, 2, ";e;sy",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue msgptr, field_name;
        if(!clips::UDFNthArgument(udfc, 1, clips::EXTERNAL_ADDRESS_BIT, &msgptr) ||
           !clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &field_name)) {
          SPDLOG_ERROR("pb-field-list: unexpected types, expected addr;lex");
          clips::UDFThrowError(udfc);
          return;
        }
        *out =
            instance->clips_pb_field_list(msgptr.externalAddressValue->contents,
                                          field_name.lexemeValue->contents);
      },
      "clips_pb_field_list", this);
  functions_.push_back("pb-field-list");

  clips::AddUDF(
      clips_, "pb-field-is-list", "b", 2, 2, ";e;sy",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue msgptr, field_name;
        if(!clips::UDFNthArgument(udfc, 1, clips::EXTERNAL_ADDRESS_BIT, &msgptr) ||
           !clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &field_name)) {
          SPDLOG_ERROR("pb-field-is-list: unexpected types, expected addr;lex");
          clips::UDFThrowError(udfc);
          return;
        }
        *out = instance->clips_pb_field_is_list(
            msgptr.externalAddressValue->contents,
            field_name.lexemeValue->contents);
      },
      "clips_pb_field_is_list", this);
  functions_.push_back("pb-field-is-list");

  clips::AddUDF(
      clips_, "pb-create", "e", 1, 1, ";sy",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue full_name;
        if(!clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &full_name)) {
          SPDLOG_ERROR("pb-create: unexpected types, expected lex");
          clips::UDFThrowError(udfc);
          return;
        }
        *out = instance->clips_pb_create(full_name.lexemeValue->contents);
      },
      "clips_pb_create", this);
  functions_.push_back("pb-create");

  clips::AddUDF(
      clips_, "pb-destroy", "v", 1, 1, ";e",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue */*out*/) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue msgptr;
        if(!clips::UDFNthArgument(udfc, 1, clips::EXTERNAL_ADDRESS_BIT, &msgptr)) {
          SPDLOG_ERROR("pb-destroy: unexpected types, expected addr");
          clips::UDFThrowError(udfc);
          return;
        }
        instance->clips_pb_destroy(msgptr.externalAddressValue->contents);
      },
      "clips_pb_destroy", this);
  functions_.push_back("pb-destroy");

  clips::AddUDF(
      clips_, "pb-ref", "e", 1, 1, ";e",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue msgptr;
        if(!clips::UDFNthArgument(udfc, 1, clips::EXTERNAL_ADDRESS_BIT, &msgptr)) {
          SPDLOG_ERROR("pb-ref: unexpected types, expected addr");
          clips::UDFThrowError(udfc);
          return;
        }
        *out = instance->clips_pb_ref(msgptr.externalAddressValue->contents);
      },
      "clips_pb_ref", this);
  functions_.push_back("pb-ref");

  clips::AddUDF(
      clips_, "pb-set-field", "v", 3, 3, ";e;sy;*",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue msgptr, field_name, value;
        if(!clips::UDFNthArgument(udfc, 1, clips::EXTERNAL_ADDRESS_BIT, &msgptr) ||
           !clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &field_name) ||
           !clips::UDFNthArgument(udfc, 3, ANY_TYPE_BITS, &value)) {
          SPDLOG_ERROR("pb-set-field: unexpected types, expected addr;lex;*");
          clips::UDFThrowError(udfc);
          return;
        }
        instance->clips_pb_set_field(msgptr.externalAddressValue->contents,
                                     field_name.lexemeValue->contents, value);
      },
      "clips_pb_set_field", this);
  functions_.push_back("pb-set-field");

  clips::AddUDF(
      clips_, "pb-add-list", "v", 3, 3, ";e;sy;*",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue msgptr, field_name, value;
        if(!clips::UDFNthArgument(udfc, 1, clips::EXTERNAL_ADDRESS_BIT, &msgptr) ||
           !clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &field_name) ||
           !clips::UDFNthArgument(udfc, 3, ANY_TYPE_BITS, &value)) {
          SPDLOG_ERROR("pb-add-list: unexpected types, expected addr;lex;*");
          clips::UDFThrowError(udfc);
          return;
        }
        instance->clips_pb_add_list(msgptr.externalAddressValue->contents,
                                    field_name.lexemeValue->contents, value);
      },
      "clips_pb_add_list", this);
  functions_.push_back("pb-add-list");

  clips::AddUDF(
      clips_, "pb-send", "v", 2, 2, ";l;e",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue client_id, msgptr;
        if (!clips::UDFNthArgument(udfc, 1, clips::INTEGER_BIT, &client_id) ||
            !clips::UDFNthArgument(udfc, 2, clips::EXTERNAL_ADDRESS_BIT,
                                   &msgptr)) {
          SPDLOG_ERROR("pb-send: unexpected types, expected int;addr");
          clips::UDFThrowError(udfc);
          return;
        }
        instance->clips_pb_send(client_id.integerValue->contents,
                                msgptr.externalAddressValue->contents);
      },
      "clips_pb_send", this);
  functions_.push_back("pb-send");

  clips::AddUDF(
      clips_, "pb-tostring", "sy", 1, 1, ";e",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue msgptr;
        if (!clips::UDFNthArgument(udfc, 1, clips::EXTERNAL_ADDRESS_BIT,
                                   &msgptr)) {
          SPDLOG_ERROR("pb-tostring: unexpected types, expected addr");
          out->lexemeValue = clips::CreateSymbol(env, "INVALID-MESSAGE");
          clips::UDFThrowError(udfc);
          return;
        }
        out->lexemeValue = clips::CreateString(
            env,
            instance->clips_pb_tostring(msgptr.externalAddressValue->contents)
                .c_str());
      },
      "clips_pb_tostring", this);
  functions_.push_back("pb-tostring");

  clips::AddUDF(
      clips_, "pb-server-enable", "v", 1, 1, ";l",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue port;
        if (!clips::UDFNthArgument(udfc, 1, clips::INTEGER_BIT, &port)) {
          SPDLOG_ERROR("pb-server-enable: unexpected types, expected int");
          clips::UDFThrowError(udfc);
          return;
        }
        instance->enable_server(port.integerValue->contents);
      },
      "enable_server", this);
  functions_.push_back("pb-server-enable");

  clips::AddUDF(
      clips_, "pb-server-disable", "v", 0, 0, "",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        instance->disable_server();
      },
      "disable_server", this);
  functions_.push_back("pb-server-disable");

  clips::AddUDF(
      clips_, "pb-peer-create", "l", 2, 2, ";s;l",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue address, port;
        if (!clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &address) ||
            !clips::UDFNthArgument(udfc, 2, clips::INTEGER_BIT, &port)) {
          SPDLOG_ERROR("pb-peer-create: unexpected types, expected lex;int");
          clips::UDFThrowError(udfc);
          return;
        }
        out->integerValue = clips::CreateInteger(
            env, instance->clips_pb_peer_create(address.lexemeValue->contents,
                                                port.integerValue->contents));
      },
      "clips_pb_peer_create", this);
  functions_.push_back("pb-peer-create");

  clips::AddUDF(
      clips_, "pb-peer-create-local", "l", 3, 3, ";sy;l;l",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue address, send_port, recv_port;
        if (!clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &address) ||
            !clips::UDFNthArgument(udfc, 2, clips::INTEGER_BIT, &send_port) ||
            !clips::UDFNthArgument(udfc, 3, clips::INTEGER_BIT, &recv_port)) {
          SPDLOG_ERROR("pb-peer-create-local: unexpected types, expected lex;int;int");
          clips::UDFThrowError(udfc);
          return;
        }
        out->integerValue =
            clips::CreateInteger(env, instance->clips_pb_peer_create_local(
                                          address.lexemeValue->contents,
                                          send_port.integerValue->contents,
                                          recv_port.integerValue->contents));
      },
      "clips_pb_peer_create_local", this);
  functions_.push_back("pb-peer-create-local");

  clips::AddUDF(
      clips_, "pb-peer-create-crypto", "l", 4, 4, ";sy;l;sy;sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue address, port, crypto_key, cipher;
        if (!clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &address) ||
            !clips::UDFNthArgument(udfc, 2, clips::INTEGER_BIT, &port) ||
            !clips::UDFNthArgument(udfc, 4, LEXEME_BITS, &crypto_key) ||
            !clips::UDFNthArgument(udfc, 5, LEXEME_BITS, &cipher)) {
          SPDLOG_ERROR("pb-peer-create-crypto: unexpected types, expected lex;int;lex,lex");
          clips::UDFThrowError(udfc);
          return;
        }
        out->integerValue = clips::CreateInteger(
            env, instance->clips_pb_peer_create_crypto(
                     address.lexemeValue->contents, port.integerValue->contents,
                     crypto_key.lexemeValue->contents,
                     cipher.lexemeValue->contents));
      },
      "clips_pb_peer_create_crypto", this);
  functions_.push_back("pb-peer-create-crypto");

  clips::AddUDF(
      clips_, "pb-peer-create-local-crypto", "l", 5, 5, ";sy;l;l;sy;sy",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue address, send_port, recv_port, crypto_key,
            cipher;
        if (!clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &address) ||
            !clips::UDFNthArgument(udfc, 2, clips::INTEGER_BIT, &send_port) ||
            !clips::UDFNthArgument(udfc, 3, clips::INTEGER_BIT, &recv_port) ||
            !clips::UDFNthArgument(udfc, 4, LEXEME_BITS, &crypto_key) ||
            !clips::UDFNthArgument(udfc, 5, LEXEME_BITS, &cipher)) {
            SPDLOG_ERROR("pb-peer-create-local-crypto: unexpected types, expected lex;int;int,lex,lex");
          clips::UDFThrowError(udfc);
          return;
        }
        out->integerValue = clips::CreateInteger(
            env,
            instance->clips_pb_peer_create_local_crypto(
                address.lexemeValue->contents, send_port.integerValue->contents,
                recv_port.integerValue->contents,
                crypto_key.lexemeValue->contents,
                cipher.lexemeValue->contents));
      },
      "clips_pb_peer_create_local_crypto", this);
  functions_.push_back("pb-peer-create-local-crypto");

  clips::AddUDF(
      clips_, "pb-peer-destroy", "v", 1, 1, ";l",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue peer_id;
        if (!clips::UDFNthArgument(udfc, 1, clips::INTEGER_BIT, &peer_id)) {
            SPDLOG_ERROR("pb-peer-destroy: unexpected types, expected int");
          clips::UDFThrowError(udfc);
          return;
        }
        instance->clips_pb_peer_destroy(peer_id.integerValue->contents);
      },
      "clips_pb_peer_destroy", this);
  functions_.push_back("pb-peer-destroy");

  clips::AddUDF(
      clips_, "pb-peer-setup-crypto", "v", 3, 3, ";l;sy;sy",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue peer_id, crypto_key, cipher;
        if (!clips::UDFNthArgument(udfc, 1, clips::INTEGER_BIT, &peer_id) ||
            !clips::UDFNthArgument(udfc, 2, LEXEME_BITS, &crypto_key) ||
            !clips::UDFNthArgument(udfc, 3, LEXEME_BITS, &cipher)) {
            SPDLOG_ERROR("pb-peer-setup-crypto: unexpected types, expected int;lex;lex");
          clips::UDFThrowError(udfc);
          return;
        }
        instance->clips_pb_peer_setup_crypto(peer_id.integerValue->contents,
                                             crypto_key.lexemeValue->contents,
                                             cipher.lexemeValue->contents);
      },
      "clips_pb_peer_setup_crypto", this);
  functions_.push_back("pb-peer-setup-crypto");

  clips::AddUDF(
      clips_, "pb-broadcast", "v", 2, 2, ";l;e",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue peer_id, msgptr;
        if (!clips::UDFNthArgument(udfc, 1, clips::INTEGER_BIT, &peer_id) ||
            !clips::UDFNthArgument(udfc, 2, clips::EXTERNAL_ADDRESS_BIT,
                                   &msgptr)) {
            SPDLOG_ERROR("pb-broadcast: unexpected types, expected int;addr");
          clips::UDFThrowError(udfc);
          return;
        }
        instance->clips_pb_broadcast(peer_id.integerValue->contents,
                                     msgptr.externalAddressValue->contents);
      },
      "clips_pb_broadcast", this);
  functions_.push_back("pb-broadcast");

  clips::AddUDF(
      clips_, "pb-connect", "l", 2, 2, ";sy;l",
      [](clips::Environment *env, clips::UDFContext *udfc,
         clips::UDFValue *out) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue host, port;
        if (!clips::UDFNthArgument(udfc, 1, LEXEME_BITS, &host) ||
            !clips::UDFNthArgument(udfc, 2, clips::INTEGER_BIT, &port)) {
            SPDLOG_ERROR("pb-connect: unexpected types, expected lex;int");
          clips::UDFThrowError(udfc);
          return;
        }
        out->integerValue = clips::CreateInteger(
            env, instance->clips_pb_client_connect(
                     host.lexemeValue->contents, port.integerValue->contents));
      },
      "clips_pb_client_connect", this);
  functions_.push_back("pb-connect");

  clips::AddUDF(
      clips_, "pb-disconnect", "v", 1, 1, ";l",
      [](clips::Environment * /*env*/, clips::UDFContext *udfc,
         clips::UDFValue * /*out*/) {
        ClipsProtobufCommunicator *instance =
            static_cast<ClipsProtobufCommunicator *>(udfc->context);
        clips::UDFValue peer_id;
        if (!clips::UDFNthArgument(udfc, 1, clips::INTEGER_BIT, &peer_id)) {
          SPDLOG_ERROR("pb-disconnect: unexpected types, expected int");
          clips::UDFThrowError(udfc);
          return;
        }
        instance->clips_pb_disconnect(peer_id.integerValue->contents);
      },
      "clips_pb_disconnect", this);
  functions_.push_back("pb-disconnect");
}

/** Enable protobuf stream server.
 * @param port TCP port to listen on for connections
 */
void ClipsProtobufCommunicator::enable_server(int port) {
  if ((port > 0) && !server_) {
    server_ = new protobuf_comm::ProtobufStreamServer(port, message_register_);

    server_->signal_connected().connect(
        boost::bind(&ClipsProtobufCommunicator::handle_server_client_connected,
                    this, _1, _2));
    server_->signal_disconnected().connect(boost::bind(
        &ClipsProtobufCommunicator::handle_server_client_disconnected, this, _1,
        _2));
    server_->signal_received().connect(
        boost::bind(&ClipsProtobufCommunicator::handle_server_client_msg, this,
                    _1, _2, _3, _4));
    server_->signal_receive_failed().connect(
        boost::bind(&ClipsProtobufCommunicator::handle_server_client_fail, this,
                    _1, _2, _3, _4));
  }
}

/** Disable protobu stream server. */
void ClipsProtobufCommunicator::disable_server() {
  delete server_;
  server_ = NULL;
}

/** Enable protobuf peer.
 * @param address IP address to send messages to
 * @param send_port UDP port to send messages to
 * @param recv_port UDP port to receive messages on, 0 to use the same as the @p
 * send_port
 * @param crypto_key encryption key
 * @param cipher cipher suite, see BufferEncryptor for supported types
 * @return peer identifier
 */
long int ClipsProtobufCommunicator::clips_pb_peer_create_local_crypto(
    std::string address, int send_port, int recv_port, std::string crypto_key,
    std::string cipher) {
  if (recv_port <= 0)
    recv_port = send_port;

  if (send_port > 0) {
    protobuf_comm::ProtobufBroadcastPeer *peer =
        new protobuf_comm::ProtobufBroadcastPeer(address, send_port, recv_port,
                                                 message_register_, crypto_key,
                                                 cipher);

    long int peer_id;
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      peer_id = ++next_client_id_;
      peers_[peer_id] = peer;
    }

    peer->signal_received().connect(
        boost::bind(&ClipsProtobufCommunicator::handle_peer_msg, this, peer_id,
                    _1, _2, _3, _4));
    peer->signal_recv_error().connect(
        boost::bind(&ClipsProtobufCommunicator::handle_peer_recv_error, this,
                    peer_id, _1, _2));
    peer->signal_send_error().connect(boost::bind(
        &ClipsProtobufCommunicator::handle_peer_send_error, this, peer_id, _1));

    return peer_id;
  } else {
    return 0;
  }
}

/** Enable protobuf peer.
 * @param address IP address to send messages to
 * @param port UDP port to send and receive messages
 * @param crypto_key encryption key
 * @param cipher cipher suite, see BufferEncryptor for supported types
 * @return peer identifier
 */
long int ClipsProtobufCommunicator::clips_pb_peer_create_crypto(
    std::string address, int port, std::string crypto_key, std::string cipher) {
  return clips_pb_peer_create_local_crypto(address, port, port, crypto_key,
                                           cipher);
}

/** Enable protobuf peer.
 * @param address IP address to send messages to
 * @param port UDP port to send and receive messages
 * @return peer identifier
 */
long int ClipsProtobufCommunicator::clips_pb_peer_create(std::string address,
                                                         int port) {
  return clips_pb_peer_create_local_crypto(address, port, port);
}

/** Enable protobuf peer.
 * @param address IP address to send messages to
 * @param send_port UDP port to send messages to
 * @param recv_port UDP port to receive messages on, 0 to use the same as the @p
 * send_port
 * @return peer identifier
 */
long int ClipsProtobufCommunicator::clips_pb_peer_create_local(
    std::string address, int send_port, int recv_port) {
  return clips_pb_peer_create_local_crypto(address, send_port, recv_port);
}

/** Disable peer.
 * @param peer_id ID of the peer to destroy
 */
void ClipsProtobufCommunicator::clips_pb_peer_destroy(long int peer_id) {
  if (peers_.find(peer_id) != peers_.end()) {
    delete peers_[peer_id];
    peers_.erase(peer_id);
  }
}

/** Setup crypto for peer.
 * @param peer_id ID of the peer to destroy
 * @param crypto_key encryption key
 * @param cipher cipher suite, see BufferEncryptor for supported types
 */
void ClipsProtobufCommunicator::clips_pb_peer_setup_crypto(
    long int peer_id, std::string crypto_key, std::string cipher) {
  if (peers_.find(peer_id) != peers_.end()) {
    peers_[peer_id]->setup_crypto(crypto_key, cipher);
  }
}

/** Register a new message type.
 * @param full_name full name of type to register
 * @return true if the type was successfully registered, false otherwise
 */
clips::UDFValue
ClipsProtobufCommunicator::clips_pb_register_type(std::string full_name) {
  clips::UDFValue res;
  try {
    message_register_->add_message_type(full_name);
    res.lexemeValue = clips::CreateBoolean(clips_, true);
    return res;
  } catch (std::runtime_error &e) {
    SPDLOG_ERROR("CLIPS-Protobuf: Registering type {} failed: {}", full_name,
                 e.what());
    res.lexemeValue = clips::CreateBoolean(clips_, false);
    return res;
  }
}

clips::UDFValue
ClipsProtobufCommunicator::clips_pb_create(std::string full_name) {
  clips::UDFValue res;
  try {
    std::shared_ptr<google::protobuf::Message> m =
        message_register_->new_message_for(full_name);
    res.externalAddressValue = clips::CreateCExternalAddress(clips_, new std::shared_ptr<google::protobuf::Message>(m));
    return res;
  } catch (std::runtime_error &e) {
    SPDLOG_WARN("CLIPS-Protobuf: Cannot create message of type {}: {}",
                full_name, e.what());
    res.externalAddressValue = clips::CreateCExternalAddress(clips_, new std::shared_ptr<google::protobuf::Message>());
    return res;
  }
}

clips::UDFValue ClipsProtobufCommunicator::clips_pb_ref(void *msgptr) {
  clips::UDFValue res;
  std::shared_ptr<google::protobuf::Message> *m =
      static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!*m) {
    res.externalAddressValue = clips::CreateCExternalAddress(
            clips_, new std::shared_ptr<google::protobuf::Message>());
    return res;
  }
  res.externalAddressValue = clips::CreateCExternalAddress(clips_, new std::shared_ptr<google::protobuf::Message>(*m));
  return res;
}

void ClipsProtobufCommunicator::clips_pb_destroy(void *msgptr) {
  std::shared_ptr<google::protobuf::Message> *m =
      static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!*m)
    return;

  delete m;
}

clips::UDFValue ClipsProtobufCommunicator::clips_pb_field_names(void *msgptr) {
  std::shared_ptr<google::protobuf::Message> *m =
      static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  clips::UDFValue field_names;
  field_names.begin = 0;
  field_names.range = -1;

  if (!*m)
    return field_names;

  const Descriptor *desc = (*m)->GetDescriptor();
  const int field_count = desc->field_count();
  std::string out_str = "";
  for (int i = 0; i < field_count; ++i) {
    out_str += " " + desc->field(i)->name();
  }
  field_names.multifieldValue =
      clips::StringToMultifield(clips_, out_str.c_str());
  return field_names;
}

clips::UDFValue
ClipsProtobufCommunicator::clips_pb_field_type(void *msgptr,
                                               std::string field_name) {
  std::shared_ptr<google::protobuf::Message> *m =
      static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  clips::UDFValue res;
  if (!*m) {
    res.lexemeValue = clips::CreateSymbol(clips_, "INVALID-MESSAGE");
    return res;
  }

  const Descriptor *desc = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (!field) {
    res.lexemeValue = clips::CreateSymbol(clips_, "DOES-NOT-EXIST");
    return res;
  }

  switch (field->type()) {
  case FieldDescriptor::TYPE_DOUBLE:
    res.lexemeValue = clips::CreateSymbol(clips_, "DOUBLE");
    break;
  case FieldDescriptor::TYPE_FLOAT:
    res.lexemeValue = clips::CreateSymbol(clips_, "FLOAT");
    break;
  case FieldDescriptor::TYPE_INT64:
    res.lexemeValue = clips::CreateSymbol(clips_, "INT64");
    break;
  case FieldDescriptor::TYPE_UINT64:
    res.lexemeValue = clips::CreateSymbol(clips_, "UINT64");
    break;
  case FieldDescriptor::TYPE_INT32:
    res.lexemeValue = clips::CreateSymbol(clips_, "INT32");
    break;
  case FieldDescriptor::TYPE_FIXED64:
    res.lexemeValue = clips::CreateSymbol(clips_, "FIXED64");
    break;
  case FieldDescriptor::TYPE_FIXED32:
    res.lexemeValue = clips::CreateSymbol(clips_, "FIXED32");
    break;
  case FieldDescriptor::TYPE_BOOL:
    res.lexemeValue = clips::CreateSymbol(clips_, "BOOL");
    break;
  case FieldDescriptor::TYPE_STRING:
    res.lexemeValue = clips::CreateSymbol(clips_, "STRING");
    break;
  case FieldDescriptor::TYPE_MESSAGE:
    res.lexemeValue = clips::CreateSymbol(clips_, "MESSAGE");
    break;
  case FieldDescriptor::TYPE_BYTES:
    res.lexemeValue = clips::CreateSymbol(clips_, "BYTES");
    break;
  case FieldDescriptor::TYPE_UINT32:
    res.lexemeValue = clips::CreateSymbol(clips_, "UINT32");
    break;
  case FieldDescriptor::TYPE_ENUM:
    res.lexemeValue = clips::CreateSymbol(clips_, "ENUM");
    break;
  case FieldDescriptor::TYPE_SFIXED32:
    res.lexemeValue = clips::CreateSymbol(clips_, "SFIXED32");
    break;
  case FieldDescriptor::TYPE_SFIXED64:
    res.lexemeValue = clips::CreateSymbol(clips_, "SFIXED64");
    break;
  case FieldDescriptor::TYPE_SINT32:
    res.lexemeValue = clips::CreateSymbol(clips_, "SINT32");
    break;
  case FieldDescriptor::TYPE_SINT64:
    res.lexemeValue = clips::CreateSymbol(clips_, "SINT64");
    break;
  default:
    res.lexemeValue = clips::CreateSymbol(clips_, "UNKNOWN");
    break;
  }
  return res;
}

clips::UDFValue
ClipsProtobufCommunicator::clips_pb_has_field(void *msgptr,
                                              std::string field_name) {
  clips::UDFValue res;
  std::shared_ptr<google::protobuf::Message> *m =
      static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!*m) {
    res.lexemeValue = clips::CreateBoolean(clips_, false);
    return res;
  }

  const Descriptor *desc = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (!field) {
    res.lexemeValue = clips::CreateBoolean(clips_, false);
    return res;
  }

  const Reflection *refl = (*m)->GetReflection();

  if (field->is_repeated()) {
    res.lexemeValue = clips::CreateBoolean(
                               clips_, (refl->FieldSize(**m, field) > 0));
  } else {
    res.lexemeValue = clips::CreateBoolean(
                               clips_, refl->HasField(**m, field));
  }
  return res;
}

clips::UDFValue
ClipsProtobufCommunicator::clips_pb_field_label(void *msgptr,
                                                std::string field_name) {
  clips::UDFValue res;
  std::shared_ptr<google::protobuf::Message> *m =
      static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!*m) {
    res.lexemeValue = clips::CreateSymbol(clips_, "INVALID-MESSAGE");
    return res;
  }

  const Descriptor *desc = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (!field) {
    res.lexemeValue = clips::CreateSymbol(clips_, "DOES-NOT-EXIST");
    return res;
  }
  switch (field->label()) {
  case FieldDescriptor::LABEL_OPTIONAL:
    res.lexemeValue = clips::CreateSymbol(clips_, "OPTIONAL");
    return res;
  case FieldDescriptor::LABEL_REQUIRED:
    res.lexemeValue = clips::CreateSymbol(clips_, "REQUIRED");
    return res;
  case FieldDescriptor::LABEL_REPEATED:
    res.lexemeValue = clips::CreateSymbol(clips_, "REPEATED");
    return res;
  default:
    res.lexemeValue = clips::CreateSymbol(clips_, "UNKNOWN");
    return res;
  }
}

clips::UDFValue
ClipsProtobufCommunicator::clips_pb_field_value(void *msgptr,
                                                std::string field_name) {
  clips::UDFValue res;
  std::shared_ptr<google::protobuf::Message> *m =
      static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m && *m)) {
    SPDLOG_WARN("CLIPS-Protobuf: Invalid message when setting {}", field_name);
    res.lexemeValue = clips::CreateSymbol(clips_, "INVALID-MESSAGE");
    return res;
  }

  const Descriptor *desc = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (!field) {
    SPDLOG_WARN("CLIPS-Protobuf: Field {} of {} does not exist", field_name,
                (*m)->GetTypeName());
    res.lexemeValue = clips::CreateSymbol(clips_, "DOES-NOT-EXIST");
    return res;
  }
  const Reflection *refl = (*m)->GetReflection();
  if (field->type() != FieldDescriptor::TYPE_MESSAGE &&
      !refl->HasField(**m, field)) {
    SPDLOG_WARN("CLIPS-Protobuf: Field {} of {} not set", field_name,
                (*m)->GetTypeName());
    res.lexemeValue = clips::CreateSymbol(clips_, "NOT-SET");
    return res;
  }
  switch (field->type()) {
  case FieldDescriptor::TYPE_DOUBLE:
        res.floatValue = clips::CreateFloat(clips_, refl->GetDouble(**m, field));
		return res;
  case FieldDescriptor::TYPE_FLOAT:
        res.floatValue = clips::CreateFloat(clips_, refl->GetFloat(**m, field));
		return res;
  case FieldDescriptor::TYPE_INT64:
        res.integerValue = clips::CreateInteger(clips_, refl->GetInt64(**m, field));
		return res;
  case FieldDescriptor::TYPE_UINT64:
        res.integerValue = clips::CreateInteger(clips_, (long int)refl->GetUInt64(**m, field));
		return res;
  case FieldDescriptor::TYPE_INT32:
        res.integerValue = clips::CreateInteger(clips_, refl->GetInt32(**m, field));
		return res;
  case FieldDescriptor::TYPE_FIXED64:
       res.integerValue = clips::CreateInteger(clips_, (long int)refl->GetUInt64(**m, field));
		return res;
  case FieldDescriptor::TYPE_FIXED32:
       res.integerValue = clips::CreateInteger(clips_, refl->GetUInt32(**m, field));
		return res;
  case FieldDescriptor::TYPE_BOOL:
       res.lexemeValue = clips::CreateBoolean(clips_, refl->GetBool(**m, field));
		return res;
  case FieldDescriptor::TYPE_BYTES:
       res.lexemeValue = clips::CreateString(clips_, (char *)"BYTES");
		return res;
  case FieldDescriptor::TYPE_STRING:
       res.lexemeValue = clips::CreateString(clips_, refl->GetString(**m, field).c_str());
		return res;
  case FieldDescriptor::TYPE_MESSAGE: {
    const google::protobuf::Message &mfield = refl->GetMessage(**m, field);
    google::protobuf::Message *mcopy = mfield.New();
    mcopy->CopyFrom(mfield);
    void *ptr = new std::shared_ptr<google::protobuf::Message>(mcopy);
       res.externalAddressValue = clips::CreateCExternalAddress(clips_, ptr);
		return res;
  }
  case FieldDescriptor::TYPE_UINT32:
       res.integerValue = clips::CreateInteger(clips_, refl->GetUInt32(**m, field));
		return res;
  case FieldDescriptor::TYPE_ENUM:
       res.lexemeValue = clips::CreateSymbol(clips_, refl->GetEnum(**m, field)->name().c_str());
		return res;
  case FieldDescriptor::TYPE_SFIXED32:
       res.integerValue = clips::CreateInteger(clips_, refl->GetInt32(**m, field));
		return res;
  case FieldDescriptor::TYPE_SFIXED64:
       res.integerValue = clips::CreateInteger(clips_, refl->GetInt64(**m, field));
		return res;
  case FieldDescriptor::TYPE_SINT32:
       res.integerValue = clips::CreateInteger(clips_, refl->GetInt32(**m, field));
		return res;
  case FieldDescriptor::TYPE_SINT64:
       res.integerValue = clips::CreateInteger(clips_, refl->GetInt64(**m, field));
		return res;
  default:
    throw std::logic_error("Unknown protobuf field type encountered");
  }
  return res;
}

void ClipsProtobufCommunicator::clips_pb_set_field(void *msgptr,
                                                   std::string field_name,
                                                   clips::UDFValue value) {
  std::shared_ptr<google::protobuf::Message> *m =
      static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m && *m))
    return;

  const Descriptor *desc = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (!field) {
    SPDLOG_WARN("CLIPS-Protobuf: Could not find field {}", field_name);
    return;
  }
  const Reflection *refl = (*m)->GetReflection();

  try {
    switch (field->type()) {
    case FieldDescriptor::TYPE_DOUBLE:
      refl->SetDouble(m->get(), field, value.floatValue->contents);
      break;
    case FieldDescriptor::TYPE_FLOAT:
      refl->SetFloat(m->get(), field, value.floatValue->contents);
      break;
    case FieldDescriptor::TYPE_SFIXED64:
    case FieldDescriptor::TYPE_SINT64:
    case FieldDescriptor::TYPE_INT64:
      refl->SetInt64(m->get(), field, value.integerValue->contents);
      break;
    case FieldDescriptor::TYPE_FIXED64:
    case FieldDescriptor::TYPE_UINT64:
      refl->SetUInt64(m->get(), field, value.integerValue->contents);
      break;
    case FieldDescriptor::TYPE_SFIXED32:
    case FieldDescriptor::TYPE_SINT32:
    case FieldDescriptor::TYPE_INT32:
      refl->SetInt32(m->get(), field, value.integerValue->contents);
      break;
    case FieldDescriptor::TYPE_BOOL:
      refl->SetBool(m->get(), field, std::strcmp(value.lexemeValue->contents, "TRUE") == 0);
      break;
    case FieldDescriptor::TYPE_STRING:
      refl->SetString(m->get(), field, value.lexemeValue->contents);
      break;
    case FieldDescriptor::TYPE_MESSAGE: {
      std::shared_ptr<google::protobuf::Message> *mfrom =
          static_cast<std::shared_ptr<google::protobuf::Message> *>(
              value.externalAddressValue->contents);
      Message *mut_msg = refl->MutableMessage(m->get(), field);
      mut_msg->CopyFrom(**mfrom);
      delete mfrom;
    } break;
    case FieldDescriptor::TYPE_BYTES:
      break;
    case FieldDescriptor::TYPE_FIXED32:
    case FieldDescriptor::TYPE_UINT32:
      refl->SetUInt32(m->get(), field, value.integerValue->contents);
      break;
    case FieldDescriptor::TYPE_ENUM: {
      const EnumDescriptor *enumdesc = field->enum_type();
      const EnumValueDescriptor *enumval =
          enumdesc->FindValueByName(value.lexemeValue->contents);
      if (enumval) {
        refl->SetEnum(m->get(), field, enumval);
      } else {
        SPDLOG_WARN(
            "CLIPS-Protobuf: {}: cannot set invalid enum value '{}' on '{}'",
            (*m)->GetTypeName(), value.lexemeValue->contents, field_name);
      }
    } break;
    default:
      throw std::logic_error("Unknown protobuf field type encountered");
    }
  } catch (std::logic_error &e) {
    SPDLOG_WARN("CLIPS-Protobuf: Failed to set field {} of {}: {} (type {}, as "
                "string {})",
                field_name, (*m)->GetTypeName(), e.what(),
                value.lexemeValue->header.type, to_string(value));
  }
}

void ClipsProtobufCommunicator::clips_pb_add_list(void *msgptr,
                                                  std::string field_name,
                                                  clips::UDFValue value) {
  std::shared_ptr<google::protobuf::Message> *m =
      static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m && *m))
    return;

  const Descriptor *desc = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (!field) {
    SPDLOG_WARN("CLIPS-Protobuf: Could not find field {}", field_name);
    return;
  }
  const Reflection *refl = (*m)->GetReflection();

  try {
    switch (field->type()) {
    case FieldDescriptor::TYPE_DOUBLE:
      refl->AddDouble(m->get(), field, value.floatValue->contents);
      break;
    case FieldDescriptor::TYPE_FLOAT:
      refl->AddFloat(m->get(), field, value.floatValue->contents);
      break;
    case FieldDescriptor::TYPE_SFIXED64:
    case FieldDescriptor::TYPE_SINT64:
    case FieldDescriptor::TYPE_INT64:
      refl->AddInt64(m->get(), field, value.integerValue->contents);
      break;
    case FieldDescriptor::TYPE_FIXED64:
    case FieldDescriptor::TYPE_UINT64:
      refl->AddUInt64(m->get(), field, (long int)value.integerValue->contents);
      break;
    case FieldDescriptor::TYPE_SFIXED32:
    case FieldDescriptor::TYPE_SINT32:
    case FieldDescriptor::TYPE_INT32:
      refl->AddInt32(m->get(), field, value.integerValue->contents);
      break;
    case FieldDescriptor::TYPE_BOOL:
      refl->AddBool(m->get(), field, std::strcmp(value.lexemeValue->contents, "TRUE") == 0);
      break;
    case FieldDescriptor::TYPE_STRING:
      refl->AddString(m->get(), field, value.lexemeValue->contents);
      break;
    case FieldDescriptor::TYPE_MESSAGE: {
      std::shared_ptr<google::protobuf::Message> *mfrom =
          static_cast<std::shared_ptr<google::protobuf::Message> *>(
              value.externalAddressValue->contents);
      Message *new_msg = refl->AddMessage(m->get(), field);
      new_msg->CopyFrom(**mfrom);
      delete mfrom;
    } break;
    case FieldDescriptor::TYPE_BYTES:
      break;
    case FieldDescriptor::TYPE_FIXED32:
    case FieldDescriptor::TYPE_UINT32:
      refl->AddUInt32(m->get(), field, value.integerValue->contents);
      break;
    case FieldDescriptor::TYPE_ENUM: {
      const EnumDescriptor *enumdesc = field->enum_type();
      const EnumValueDescriptor *enumval =
          enumdesc->FindValueByName(value.lexemeValue->contents);
      if (enumval)
        refl->AddEnum(m->get(), field, enumval);
    } break;
    default:
      throw std::logic_error("Unknown protobuf field type encountered");
    }
  } catch (std::logic_error &e) {
    SPDLOG_WARN("CLIPS-Protobuf: Failed to add field {} of {}: {}", field_name,
                (*m)->GetTypeName().c_str(), e.what());
  }
}

long int ClipsProtobufCommunicator::clips_pb_client_connect(std::string host,
                                                            int port) {
  if (port <= 0)
    return false;

  ProtobufStreamClient *client = new ProtobufStreamClient(message_register_);

  long int client_id;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    client_id = ++next_client_id_;
    clients_[client_id] = client;
  }

  client->signal_connected().connect(boost::bind(
      &ClipsProtobufCommunicator::handle_client_connected, this, client_id));
  client->signal_disconnected().connect(
      boost::bind(&ClipsProtobufCommunicator::handle_client_disconnected, this,
                  client_id, boost::asio::placeholders::error));
  client->signal_received().connect(
      boost::bind(&ClipsProtobufCommunicator::handle_client_msg, this,
                  client_id, _1, _2, _3));
  client->signal_receive_failed().connect(
      boost::bind(&ClipsProtobufCommunicator::handle_client_receive_fail, this,
                  client_id, _1, _2, _3));

  client->async_connect(host.c_str(), port);
  return client_id;
}

void ClipsProtobufCommunicator::clips_pb_send(long int client_id,
                                              void *msgptr) {
  std::shared_ptr<google::protobuf::Message> *m =
      static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m && *m)) {
    SPDLOG_WARN("CLIPS-Protobuf: Cannot send to {}: invalid message",
                client_id);
    return;
  }

  try {
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (server_ && server_clients_.find(client_id) != server_clients_.end()) {
      // printf("***** SENDING via SERVER\n");
      server_->send(server_clients_[client_id], *m);
      sig_server_sent_(server_clients_[client_id], *m);
    } else if (clients_.find(client_id) != clients_.end()) {
      // printf("***** SENDING via CLIENT\n");
      clients_[client_id]->send(*m);
      std::pair<std::string, unsigned short> &client_endpoint =
          client_endpoints_[client_id];
      sig_client_sent_(client_endpoint.first, client_endpoint.second, *m);
    } else if (peers_.find(client_id) != peers_.end()) {
      // printf("***** SENDING via CLIENT\n");
      peers_[client_id]->send(*m);
      sig_peer_sent_(client_id, *m);
    } else {
      // printf("Client ID %li is unknown, cannot send message of type %s\n",
      //     client_id, (*m)->GetTypeName().c_str());
    }
  } catch (google::protobuf::FatalException &e) {
    SPDLOG_WARN("CLIPS-Profobuf: Failed to send message of type {}: {}",
                (*m)->GetTypeName().c_str(), e.what());
  } catch (std::runtime_error &e) {
    SPDLOG_WARN("CLIPS-Profobuf: Failed to send message of type {}: {}",
                (*m)->GetTypeName().c_str(), e.what());
  }
}

std::string ClipsProtobufCommunicator::clips_pb_tostring(void *msgptr) {
  std::shared_ptr<google::protobuf::Message> *m =
      static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m && *m)) {
    SPDLOG_WARN(
        "CLIPS-Protobuf: Cannot convert message to string: invalid message");
    return "";
  }

  return (*m)->DebugString();
}

void ClipsProtobufCommunicator::clips_pb_broadcast(long int peer_id,
                                                   void *msgptr) {
  std::shared_ptr<google::protobuf::Message> *m =
      static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m && *m)) {
    SPDLOG_WARN("CLIPS-Protobuf: Cannot send broadcast: invalid message");
    return;
  }

  std::lock_guard<std::mutex> lock(map_mutex_);
  if (peers_.find(peer_id) == peers_.end())
    return;

  SPDLOG_INFO("CLIPS-Protobuf: Broadcasting {}", (*m)->GetTypeName().c_str());
  try {
    peers_[peer_id]->send(*m);
  } catch (google::protobuf::FatalException &e) {
    SPDLOG_WARN("Failed to broadcast message of type {}: {}",
                (*m)->GetTypeName(), e.what());
  } catch (std::runtime_error &e) {
    SPDLOG_WARN("Failed to broadcast message of type {}: {}",
                (*m)->GetTypeName(), e.what());
  }

  sig_peer_sent_(peer_id, *m);
}

void ClipsProtobufCommunicator::clips_pb_disconnect(long int client_id) {
  SPDLOG_INFO("CLIPS-Protobuf: Disconnecting client {}", client_id);

  try {
    std::lock_guard<std::mutex> lock(map_mutex_);

    if (server_clients_.find(client_id) != server_clients_.end()) {
      protobuf_comm::ProtobufStreamServer::ClientID srv_client =
          server_clients_[client_id];
      server_->disconnect(srv_client);
      server_clients_.erase(client_id);
      rev_server_clients_.erase(srv_client);
    } else if (clients_.find(client_id) != clients_.end()) {
      delete clients_[client_id];
      clients_.erase(client_id);
    }
  } catch (std::runtime_error &e) {
    SPDLOG_WARN("CLIPS-Protobuf: Failed to disconnect from client {}: {}",
                client_id, e.what());
  }
}

clips::UDFValue
ClipsProtobufCommunicator::clips_pb_field_list(void *msgptr,
                                               std::string field_name) {
  clips::UDFValue res;
  std::shared_ptr<google::protobuf::Message> *m =
      static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m && *m)) {
    res.lexemeValue = clips::CreateSymbol(clips_, "INVALID-MESSAGE");
    return res;
  }

  const Descriptor *desc = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (!field) {
    res.lexemeValue = clips::CreateSymbol(clips_, "DOES-NOT-EXIST");
    return res;
  }
  res.begin = 0;
  res.range = -1;
  clips::MultifieldBuilder *mb = clips::CreateMultifieldBuilder(clips_, 10);
  if (field->label() == FieldDescriptor::LABEL_REQUIRED ||
      field->label() == FieldDescriptor::LABEL_OPTIONAL) {
    clips::UDFValue aux_val = clips_pb_field_value(msgptr, field_name);
    clips::MBAppend(mb, static_cast<clips::CLIPSValue *>(aux_val.value));
    res.multifieldValue = clips::MBCreate(mb);
    MBDispose(mb);
    return res;
  }

  const Reflection *refl = (*m)->GetReflection();
  int field_size = refl->FieldSize(**m, field);
  for (int i = 0; i < field_size; ++i) {
    switch (field->type()) {
    case FieldDescriptor::TYPE_DOUBLE:
      clips::MBAppendFloat(mb, refl->GetRepeatedDouble(**m, field, i));
      break;
    case FieldDescriptor::TYPE_FLOAT:
      clips::MBAppendFloat(mb, refl->GetRepeatedFloat(**m, field, i));
      break;
    case FieldDescriptor::TYPE_UINT64:
    case FieldDescriptor::TYPE_FIXED64:
      clips::MBAppendInteger(mb,
                             (long int)refl->GetRepeatedUInt64(**m, field, i));
      break;
    case FieldDescriptor::TYPE_UINT32:
    case FieldDescriptor::TYPE_FIXED32:
      clips::MBAppendInteger(mb, refl->GetRepeatedUInt32(**m, field, i));
      break;
    case FieldDescriptor::TYPE_BOOL:
      // Booleans are represented as Symbols in CLIPS
      if (refl->GetRepeatedBool(**m, field, i)) {
        clips::MBAppendSymbol(mb, "TRUE");
      } else {
        clips::MBAppendSymbol(mb, "FALSE");
      }
      break;
    case FieldDescriptor::TYPE_STRING:
      clips::MBAppendString(mb, refl->GetRepeatedString(**m, field, i).c_str());
      break;
    case FieldDescriptor::TYPE_MESSAGE: {
      const google::protobuf::Message &msg =
          refl->GetRepeatedMessage(**m, field, i);
      google::protobuf::Message *mcopy = msg.New();
      mcopy->CopyFrom(msg);
      void *ptr = new std::shared_ptr<google::protobuf::Message>(mcopy);
      clips::MBAppendCLIPSExternalAddress(
          mb, clips::CreateCExternalAddress(clips_, ptr));
    } break;
    case FieldDescriptor::TYPE_BYTES:
      clips::MBAppendString(mb, (char *)"BYTES");
      break;
    case FieldDescriptor::TYPE_ENUM:
      clips::MBAppendSymbol(
          mb, refl->GetRepeatedEnum(**m, field, i)->name().c_str());
      break;
    case FieldDescriptor::TYPE_SFIXED32:
    case FieldDescriptor::TYPE_INT32:
    case FieldDescriptor::TYPE_SINT32:
      clips::MBAppendInteger(mb, refl->GetRepeatedInt32(**m, field, i));
      break;
    case FieldDescriptor::TYPE_SFIXED64:
    case FieldDescriptor::TYPE_SINT64:
    case FieldDescriptor::TYPE_INT64:
      clips::MBAppendInteger(mb, refl->GetRepeatedInt64(**m, field, i));
      break;
    default:
      throw std::logic_error("Unknown protobuf field type encountered");
    }
  }
  auto val = clips::MBCreate(mb);
  res .multifieldValue = val;
  MBDispose(mb);
  return res;
}

clips::UDFValue
ClipsProtobufCommunicator::clips_pb_field_is_list(void *msgptr,
                                                  std::string field_name) {
  clips::UDFValue res;
  std::shared_ptr<google::protobuf::Message> *m =
      static_cast<std::shared_ptr<google::protobuf::Message> *>(msgptr);
  if (!(m && *m)) {
    res.lexemeValue = clips::CreateBoolean(clips_, false);
    return res;
  }

  const Descriptor *desc = (*m)->GetDescriptor();
  const FieldDescriptor *field = desc->FindFieldByName(field_name);
  if (!field) {
    res.lexemeValue = clips::CreateBoolean(clips_, false);
  }
  res.lexemeValue = clips::CreateBoolean(clips_, field->is_repeated());
  return res;
}

void ClipsProtobufCommunicator::clips_assert_message(
    std::pair<std::string, unsigned short> &endpoint, uint16_t comp_id,
    uint16_t msg_type, std::shared_ptr<google::protobuf::Message> &msg,
    ClipsProtobufCommunicator::ClientType ct, long int client_id) {
  clips::FactBuilder *fact_builder =
      clips::CreateFactBuilder(clips_, "protobuf-msg");
  if (fact_builder) {
    clips::FBPutSlotString(fact_builder, "type", msg->GetTypeName().c_str());
    clips::FBPutSlotInteger(fact_builder, "comp-id", comp_id);
    clips::FBPutSlotInteger(fact_builder, "msg-type", msg_type);
    clips::FBPutSlotSymbol(fact_builder, "rcvd-via",
                           (ct == CT_PEER) ? "BROADCAST" : "STREAM");
    struct timeval tv;
    gettimeofday(&tv, 0);
    clips::FBPutSlotMultifield(
        fact_builder, "rcvd-at",
        clips::StringToMultifield(
            clips_, std::format("{} {}", tv.tv_sec, tv.tv_usec).c_str()));
    clips::FBPutSlotMultifield(
        fact_builder, "rcvd-from",
        clips::StringToMultifield(
            clips_,
            std::format("{} {}", endpoint.first, endpoint.second).c_str()));
    clips::FBPutSlotSymbol(
        fact_builder, "client-type",
        ct == CT_CLIENT ? "CLIENT" : (ct == CT_SERVER ? "SERVER" : "PEER"));
    clips::FBPutSlotInteger(fact_builder, "client-id", client_id);
    void *ptr = new std::shared_ptr<google::protobuf::Message>(msg);
    clips::FBPutSlotCLIPSExternalAddress(
        fact_builder, "ptr", clips::CreateCExternalAddress(clips_, ptr));
    clips::Fact *new_fact = clips::FBAssert(fact_builder);

    if (!new_fact) {
      SPDLOG_WARN("CLIPS-Protobuf: Asserting protobuf-msg fact failed");
      delete static_cast<std::shared_ptr<google::protobuf::Message> *>(ptr);
    }
    clips::FBDispose(fact_builder);
  } else {
    SPDLOG_WARN(
        "CLIPS-Protobuf: Did not get template, did you load protobuf.clp?");
  }
}

void ClipsProtobufCommunicator::handle_server_client_connected(
    ProtobufStreamServer::ClientID client,
    boost::asio::ip::tcp::endpoint &endpoint) {
  long int client_id = -1;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    client_id = ++next_client_id_;
    client_endpoints_[client_id] =
        std::make_pair(endpoint.address().to_string(), endpoint.port());
    server_clients_[client_id] = client;
    rev_server_clients_[client] = client_id;
  }

  std::lock_guard<std::mutex> lock(clips_mutex_);
  clips::AssertString(
      clips_,
      std::format("(protobuf-server-client-connected {} {} {})", client_id,
                  endpoint.address().to_string().c_str(), endpoint.port())
          .c_str());
}

void ClipsProtobufCommunicator::handle_server_client_disconnected(
    ProtobufStreamServer::ClientID client,
    const boost::system::error_code &/*error*/) {
  long int client_id = -1;
  {
    std::lock_guard<std::mutex> lock(map_mutex_);
    RevServerClientMap::iterator c;
    if ((c = rev_server_clients_.find(client)) != rev_server_clients_.end()) {
      client_id = c->second;
      rev_server_clients_.erase(c);
      server_clients_.erase(client_id);
    }
  }

  if (client_id >= 0) {
    std::lock_guard<std::mutex> lock(clips_mutex_);
    clips::AssertString(
        clips_,
        std::format("(protobuf-server-client-disconnected {})", client_id)
            .c_str());
  }
}

/** Handle message that came from a client.
 * @param client client ID
 * @param component_id component the message was addressed to
 * @param msg_type type of the message
 * @param msg the message
 */
void ClipsProtobufCommunicator::handle_server_client_msg(
    ProtobufStreamServer::ClientID client, uint16_t component_id,
    uint16_t msg_type, std::shared_ptr<google::protobuf::Message> msg) {
  std::lock_guard<std::mutex> lock(clips_mutex_);
  std::lock_guard<std::mutex> lock2(map_mutex_);
  RevServerClientMap::iterator c;
  if ((c = rev_server_clients_.find(client)) != rev_server_clients_.end()) {
    clips_assert_message(client_endpoints_[c->second], component_id, msg_type,
                         msg, CT_SERVER, c->second);
  }
}

/** Handle server reception failure
 * @param client client ID
 * @param component_id component the message was addressed to
 * @param msg_type type of the message
 * @param msg the message string
 */
void ClipsProtobufCommunicator::handle_server_client_fail(
    ProtobufStreamServer::ClientID client, uint16_t component_id,
    uint16_t msg_type, std::string msg) {
  std::lock_guard<std::mutex> lock(map_mutex_);
  RevServerClientMap::iterator c;
  if ((c = rev_server_clients_.find(client)) != rev_server_clients_.end()) {
    std::lock_guard<std::mutex> lock(clips_mutex_);
    clips::AssertString(
        clips_,
        std::format(
            "(protobuf-server-receive-failed (comp-id {}) (msg-type {}) "
            "(rcvd-via STREAM) (client-id {}) (message \"{}\") "
            "(rcvd-from (\"{}\" {})))",
            component_id, msg_type, c->second, msg.c_str(),
            client_endpoints_[c->second].first.c_str(),
            client_endpoints_[c->second].second)
            .c_str());
  }
}

/** Handle message that came from a peer/robot
 * @param endpoint the endpoint from which the message was received
 * @param component_id component the message was addressed to
 * @param msg_type type of the message
 * @param msg the message
 */
void ClipsProtobufCommunicator::handle_peer_msg(
    long int peer_id, boost::asio::ip::udp::endpoint &endpoint,
    uint16_t component_id, uint16_t msg_type,
    std::shared_ptr<google::protobuf::Message> msg) {
  std::lock_guard<std::mutex> lock(clips_mutex_);
  std::pair<std::string, unsigned short> endpp =
      std::make_pair(endpoint.address().to_string(), endpoint.port());
  clips_assert_message(endpp, component_id, msg_type, msg, CT_PEER, peer_id);
}

/** Handle error during peer message processing.
 * @param endpoint endpoint of incoming message
 * @param msg error message
 */
void ClipsProtobufCommunicator::handle_peer_recv_error(
    long int /*peer_id*/, boost::asio::ip::udp::endpoint &endpoint,
    std::string msg) {
  SPDLOG_WARN("CLIPS-Protobuf: Failed to receive peer message from {}:{}: {}",
              endpoint.address().to_string(), endpoint.port(), msg);
}

/** Handle error during peer message processing.
 * @param msg error message
 */
void ClipsProtobufCommunicator::handle_peer_send_error(long int /*peer_id*/,
                                                       std::string msg) {
  SPDLOG_WARN("CLIPS-Protobuf: Failed to send peer message: {}", msg);
}

void ClipsProtobufCommunicator::handle_client_connected(long int client_id) {
  std::lock_guard<std::mutex> lock(clips_mutex_);
  clips::AssertString(
      clips_, std::format("(protobuf-client-connected {})", client_id).c_str());
}

void ClipsProtobufCommunicator::handle_client_disconnected(
    long int client_id, const boost::system::error_code &/*error*/) {
  std::lock_guard<std::mutex> lock(clips_mutex_);
  clips::AssertString(
      clips_,
      std::format("(protobuf-client-disconnected {})", client_id).c_str());
}

void ClipsProtobufCommunicator::handle_client_msg(
    long int client_id, uint16_t comp_id, uint16_t msg_type,
    std::shared_ptr<google::protobuf::Message> msg) {
  std::lock_guard<std::mutex> lock(clips_mutex_);
  std::pair<std::string, unsigned short> endpp =
      std::make_pair(std::string(), 0);
  clips_assert_message(endpp, comp_id, msg_type, msg, CT_CLIENT, client_id);
}

void ClipsProtobufCommunicator::handle_client_receive_fail(long int client_id,
                                                           uint16_t comp_id,
                                                           uint16_t msg_type,
                                                           std::string msg) {
  std::lock_guard<std::mutex> lock(clips_mutex_);
  clips::AssertString(
      clips_,
      std::format("(protobuf-receive-failed (client-id {}) (rcvd-via STREAM) "
                  "(comp-id {}) (msg-type {}) (message \"{}\"))",
                  client_id, comp_id, msg_type, msg.c_str())
          .c_str());
}

std::string ClipsProtobufCommunicator::to_string(const clips::UDFValue &v) {
  switch (v.lexemeValue->header.type) {
  case VOID_TYPE:
    return "Void Type";
  case FLOAT_TYPE:
    return std::to_string(v.floatValue->contents);
  case INTEGER_TYPE:
    return std::to_string(v.integerValue->contents);
  case SYMBOL_TYPE:
  case INSTANCE_NAME_TYPE:
  case STRING_TYPE:
    return v.lexemeValue->contents;
  case INSTANCE_ADDRESS_TYPE:
  case FACT_ADDRESS_TYPE:
  case EXTERNAL_ADDRESS_TYPE:
    return boost::str(boost::format("%p") % v.externalAddressValue->contents);
  case MULTIFIELD_TYPE:
    std::string res;

    for (size_t i = v.begin; i < (v.begin + v.range); i++) {
      // TODO: actually, a type check for each field would be needed...
      res += v.multifieldValue->contents[i].lexemeValue->contents;
    }
    return res;
  }
  return "Implicit unknown type";
}

} // end namespace protobuf_clips
